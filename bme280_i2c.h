#ifndef _BME280_I2C_H_
#define _BME280_I2C_H_

#include <stdint.h>

//SDO Pin ist I2C Adresse LSB
#define BME280_PIN_SDO_LEVEL 1

#if BME280_PIN_SDO_LEVEL == 0
	#define BME280_SLAVE_ADDRESS 0xec
#else
	#define BME280_SLAVE_ADDRESS 0xee
#endif

#define BME280_I2C_FAIL 	1
#define BME280_STAT_TIMEOUT	2

uint8_t BME280_init(void);

//Filterkoeffizient des IIR Filter setzen
uint8_t BME280_set_filter(uint8_t filter_coeff);

//Stadbyzeit setzen zwischen 2 Messzyklen in "BME280_MODE_NORM"
uint8_t BME280_set_standby(uint8_t t_standby);

//Oversamplingrate setzen für den übergebenen Messungstyp
uint8_t BME280_set_measure(uint8_t oversampling, uint8_t measuretype);

//Messmodus des BME280 setzen
uint8_t BME280_set_measuremode(uint8_t measuremode);

//Messdaten aus dem Sensor lesen
// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// Returns pressure in Pa as unsigned 32 bit integer. Output value of “96386” equals 96386 Pa = 963.86 hPa
// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of “47445” represents 47445/1024 = 46.333 %RH
uint8_t BME280_readout(int32_t* temp_p, uint32_t* press_p, uint32_t* hum_p);



/*Automatically set to ‘1’ whenever a conversion is running
and back to ‘0’ when the results have been transferred
to the data registers.*/
#define BME_280_STAT_MEASURING (1<<3)	

/*Automatically set to ‘1’ when the NVM data are being
copied to image registers and back to ‘0’ when the
copying is done. The data are copied at power-on-reset
and before every conversion.*/
#define BME_280_STAT_IM_UPDATE (1<<0)

//T standby
#define BME280_TSB_0_5		0
#define BME280_TSB_62_5		1
#define BME280_TSB_125		2
#define BME280_TSB_250		3
#define BME280_TSB_500		4
#define BME280_TSB_1000		5
#define BME280_TSB_10		6
#define BME280_TSB_20		7

//Filter coefficient 
#define BME280_FILTER_OFF	0
#define BME280_FILTER_2		1
#define BME280_FILTER_4		2
#define BME280_FILTER_8		3
#define BME280_FILTER_16	4

//Oversampling
#define BME280_SKIP			0
#define BME280_OVER_1		1
#define BME280_OVER_2		2
#define BME280_OVER_4		3
#define BME280_OVER_8		4
#define BME280_OVER_16		5

//Measure Modes
#define BME280_MODE_SLEEP 	0
#define BME280_MODE_FORCE 	1
#define BME280_MODE_NORM 	3

//Measure Type
#define BME280_HUM			0
#define BME280_PRESS		1
#define BME280_TEMP			2

#endif//_BME280_I2C_H_
