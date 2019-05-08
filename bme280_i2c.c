#include <bme280_i2c.h>
#include <i2cmaster.h>

static uint16_t dig_T1;
static int16_t dig_T2;
static int16_t dig_T3;

static uint16_t dig_P1;
static int16_t dig_P2;
static int16_t dig_P3;
static int16_t dig_P4;
static int16_t dig_P5;
static int16_t dig_P6;
static int16_t dig_P7;
static int16_t dig_P8;
static int16_t dig_P9;

static uint8_t dig_H1;
static int16_t dig_H2;
static uint8_t dig_H3;
static int16_t dig_H4;
static int16_t dig_H5;
static int8_t dig_H6;

// t_fine carries fine temperature as global value
static int32_t t_fine;

static int32_t BME280_compensate_T_int32(int32_t adc_T);
static uint32_t BME280_compensate_P_int32(int32_t adc_P);
static uint32_t bme280_compensate_H_int32(int32_t adc_H);


static uint8_t BME280_write(const uint8_t* data, uint8_t bytes, uint8_t startaddress){
	uint8_t failed, i;
	failed = i2c_start(BME280_SLAVE_ADDRESS+I2C_WRITE);
	if (failed){
		return BME280_I2C_FAIL;
	}
	
	for (i = 0; i < bytes; i++){
		failed |= i2c_write(startaddress++); //Registeradresse
		failed |= i2c_write(data[i]);
		if (failed){
			i2c_stop();
			return BME280_I2C_FAIL;
		}
	}
	
	i2c_stop();
	
	return 0;
}

static uint8_t BME280_read(uint8_t* data, uint8_t bytes, uint8_t startaddress){
	uint8_t failed, i;
	failed = i2c_start(BME280_SLAVE_ADDRESS+I2C_WRITE);
	if (failed){
		return BME280_I2C_FAIL;
	}
	
	failed |= i2c_write(startaddress); //Registeradresse
	failed |= i2c_rep_start(BME280_SLAVE_ADDRESS+I2C_READ);
	if (failed){
		i2c_stop();
		return BME280_I2C_FAIL;
	}
	
	for (i = 0; i < bytes-1; i++){
		data[i] = i2c_readAck();
	}
	data[i] = i2c_readNak();
	
	i2c_stop();
	
	return 0;
}

//wartet maximal 100 I2C Buszugriffe auf das Statusbit
static uint8_t BME280_waitfor_status(uint8_t statusbit){
	uint8_t i, statusregister;
	uint8_t failed = 0;
	for (i = 0; i < 100; i++){
		failed |= BME280_read(&statusregister, 1, 0xF3);
		if (failed){
			return BME280_I2C_FAIL;
		}
		if ( 0 == (statusregister & statusbit)){
			return 0;
		}
	}
	return BME280_STAT_TIMEOUT;
}

uint8_t BME280_init(void){
	
	uint8_t calbytes_low[26];
	uint8_t calbytes_high[7];
	uint8_t failed = 0;
	uint8_t reset = 0xB6;
	
	failed |= BME280_write(&reset, 1, 0xE0);
	if (failed){
		return BME280_I2C_FAIL;
	}
	
	//Kalibierungsregister lesen
	failed |= BME280_waitfor_status(BME_280_STAT_IM_UPDATE);
	if (failed){
		return BME280_I2C_FAIL;
	}
	failed |= BME280_read(calbytes_low, 26, 0x88);
	failed |= BME280_read(calbytes_high, 7, 0xE1);
	if (failed){
		return BME280_I2C_FAIL;
	}
	
	dig_T1 = (uint16_t)((((uint16_t)((uint8_t)calbytes_low[1])) <<8) | calbytes_low[0]);	
	dig_T2 = (int16_t) ((((int16_t) ((int8_t) calbytes_low[3])) <<8) | calbytes_low[2]);	
	dig_T3 = (int16_t) ((((int16_t) ((int8_t) calbytes_low[5])) <<8) | calbytes_low[4]);	
	dig_P1 = (uint16_t)((((uint16_t)((uint8_t)calbytes_low[7])) <<8) | calbytes_low[6]);	
	dig_P2 = (int16_t) ((((int16_t) ((int8_t) calbytes_low[9])) <<8) | calbytes_low[8]);	
	dig_P3 = (int16_t) ((((int16_t) ((int8_t) calbytes_low[11])) <<8)| calbytes_low[10]);	
	dig_P4 = (int16_t) ((((int16_t) ((int8_t) calbytes_low[13])) <<8)| calbytes_low[12]);	
	dig_P5 = (int16_t) ((((int16_t) ((int8_t) calbytes_low[15])) <<8)| calbytes_low[14]);	
	dig_P6 = (int16_t) ((((int16_t) ((int8_t) calbytes_low[17])) <<8)| calbytes_low[16]);	
	dig_P7 = (int16_t) ((((int16_t) ((int8_t) calbytes_low[19])) <<8)| calbytes_low[18]);	
	dig_P8 = (int16_t) ((((int16_t) ((int8_t) calbytes_low[21])) <<8)| calbytes_low[20]);	
	dig_P9 = (int16_t) ((((int16_t) ((int8_t) calbytes_low[23])) <<8)| calbytes_low[22]);	
	dig_H1 = calbytes_low[25];
	
	dig_H2 = (int16_t)((((int16_t)((int8_t)calbytes_high[1])) <<8)| calbytes_high[0]);	
	dig_H3 = calbytes_high[2];	
	dig_H4 = (int16_t)((((int16_t)((int8_t)calbytes_high[3])) <<4) |(((uint8_t)0x0F) & calbytes_high[4]));	
	dig_H5 = (int16_t)((((int16_t)((int8_t)calbytes_high[5])) <<4) |(calbytes_high[4] >>4));	
	dig_H6 = (int8_t)calbytes_high[6];
	
	return 0;
}

uint8_t BME280_set_filter(uint8_t filter_coeff){
	uint8_t config_register;
	uint8_t failed = 0;
	
	failed |= BME280_read(&config_register, 1, 0xF5);
	if (failed){
		return BME280_I2C_FAIL;
	}
	
	config_register &= 0xE3;
	config_register |= (filter_coeff << 2);
	
	failed |= BME280_write(&config_register, 1, 0xF5);
	if (failed){
		return BME280_I2C_FAIL;
	}
	
	return 0;
}

uint8_t BME280_set_standby(uint8_t t_standby){
	uint8_t config_register;
	uint8_t failed = 0;
	
	failed |= BME280_read(&config_register, 1, 0xF5);
	if (failed){
		return BME280_I2C_FAIL;
	}
	
	config_register &= 0x1F;
	config_register |= (t_standby << 5);
	
	failed |= BME280_write(&config_register, 1, 0xF5);
	if (failed){
		return BME280_I2C_FAIL;
	}
	
	return 0;
}

uint8_t BME280_set_measure(uint8_t oversampling, uint8_t measuretype){
	uint8_t ctrl_hum, ctrl_meas;
	uint8_t failed = 0;
	
	failed |= BME280_read(&ctrl_meas, 1, 0xF4);
	if (failed){
		return BME280_I2C_FAIL;
	}
	
	switch (measuretype){
		case BME280_HUM:
			ctrl_hum = oversampling;
			failed |= BME280_write(&ctrl_hum, 1, 0xF2);
			if (failed){
				return BME280_I2C_FAIL;
			}
			break;
		case BME280_PRESS:
			ctrl_meas &= 0xE3;
			ctrl_meas |= (oversampling << 2);
			break;
		case BME280_TEMP:
			ctrl_meas &= 0x1F;
			ctrl_meas |= (oversampling << 5);
			break;
		default:
			break;
	}
	
	failed |= BME280_write(&ctrl_meas, 1, 0xF4);
	if (failed){
		return BME280_I2C_FAIL;
	}
	
	return 0;
}

uint8_t BME280_set_measuremode(uint8_t measuremode){
	uint8_t ctrl_meas;
	uint8_t failed = 0;
	
	failed |= BME280_read(&ctrl_meas, 1, 0xF4);
	if (failed){
		return BME280_I2C_FAIL;
	}
	
	ctrl_meas &= 0xFC;
	ctrl_meas |= measuremode;
	
	failed |= BME280_write(&ctrl_meas, 1, 0xF4);
	if (failed){
		return BME280_I2C_FAIL;
	}
	
	return 0;
}

uint8_t BME280_readout(int32_t* temp_p, uint32_t* press_p, uint32_t* hum_p){

	uint8_t failed = 0;
	
	//Messung triggern wenn sleep mode statt normal mode
	uint8_t ctrl_meas;
	failed |= BME280_read(&ctrl_meas, 1, 0xF4);
	if (failed){
		return BME280_I2C_FAIL;
	}
	if ((ctrl_meas & 0x03) == BME280_MODE_SLEEP){
		ctrl_meas |= BME280_MODE_FORCE;
		failed |= BME280_write(&ctrl_meas, 1, 0xF4);
		if (failed){
			return BME280_I2C_FAIL;
		}
		//warten nach anstoßen der Messung
		failed |= BME280_waitfor_status(BME_280_STAT_MEASURING);
		if (failed){
			return BME280_I2C_FAIL;
		}
	}
	
	
	uint8_t measurebytes[8];
	failed |= BME280_read(measurebytes, 8, 0xF7);
	if (failed){
		return BME280_I2C_FAIL;
	}
	
	int32_t press = (int32_t)(((uint32_t)measurebytes[0]<<12) | ((uint32_t)measurebytes[1]<<4) | (uint32_t)measurebytes[2] >> 4);
	int32_t temp  = (int32_t)(((uint32_t)measurebytes[3]<<12) | ((uint32_t)measurebytes[4]<<4) | (uint32_t)measurebytes[5] >> 4);
	int32_t hum   = (int32_t)(((uint32_t)measurebytes[6]<<8)  | ((uint32_t)measurebytes[7]));
	
	*temp_p  = BME280_compensate_T_int32(temp);
	*press_p = BME280_compensate_P_int32(press);
	*hum_p   = bme280_compensate_H_int32(hum);
	
	return 0;
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
static int32_t BME280_compensate_T_int32(int32_t adc_T){
	int32_t var1, var2, T;
	var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}
	
// Returns pressure in Pa as unsigned 32 bit integer. Output value of “96386” equals 96386 Pa = 963.86 hPa
static uint32_t BME280_compensate_P_int32(int32_t adc_P){
	int32_t var1, var2;
	uint32_t p;
	var1 = (((int32_t)t_fine)>>1) - (int32_t)64000;
	var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((int32_t)dig_P6);
	var2 = var2 + ((var1*((int32_t)dig_P5))<<1);
	var2 = (var2>>2)+(((int32_t)dig_P4)<<16);
	var1 = (((dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((int32_t)dig_P2) * var1)>>1))>>18;
	var1 =((((32768L+var1))*((int32_t)dig_P1))>>15);
	if (var1 == 0){
		return 0; // avoid exception caused by division by zero
	}
	p = (((uint32_t)(((int32_t)1048576)-adc_P)-(var2>>12)))*3125L;
	if (p < 0x80000000L){
		p = (p << 1) / ((uint32_t)var1);
	}else{
		p = (p / (uint32_t)var1) * 2;
	}
	var1 = (((int32_t)dig_P9) * ((int32_t)(((p>>3) * (p>>3))>>13)))>>12;
	var2 = (((int32_t)(p>>2)) * ((int32_t)dig_P8))>>13;
	p = (uint32_t)((int32_t)p + ((var1 + var2 + dig_P7) >> 4));
	return p;
}

// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of “47445” represents 47445/1024 = 46.333 %RH
static uint32_t bme280_compensate_H_int32(int32_t adc_H){
	int32_t v_x1_u32r;
	v_x1_u32r = (t_fine - ((int32_t)76800));
	v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1_u32r)) +
		((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) * (((v_x1_u32r *	
		((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
		((int32_t)dig_H2) + 8192) >> 14));
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400L ? 419430400L : v_x1_u32r);
	return (uint32_t)(v_x1_u32r>>12);
}