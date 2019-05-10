/*
Nokia Sensorboard
* 
* Done:
* => Read Sensor BME280
* => Write Sensor Data on Display
* => Write Sensor Data on USART (formatted for KST plotting with KST)
* 
* Pending:
* 
* => read / write EEPROM (12C)
* => read / write RTC (I2C)
* => logging function
* 
* z
*/


#define F_CPU 8000000UL                 // set the CPU clock

#include <avr/io.h>
#include "glcd/glcd.h"
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include "glcd/fonts/font5x7.h"
#include <avr/pgmspace.h>
#include "bme280_i2c.h"
#include "i2cmaster.h"

#include <string.h>
#include "fat.h"
#include "fat_config.h"
#include "partition.h"
#include "sd_raw.h"
#include "sd_raw_config.h"
#include "uart.h"


#define T_YELLOW !(PIND & (1<<PD5)) && (entprell == 0)
#define T_BLUE !(PIND & (1<<PD6)) && (entprell == 0)
#define T_GREEN !(PIND & (1<<PD2)) && (entprell == 0)
#define RELOAD_ENTPRELL 30

#define LED_EIN PORTC |= (1<<PC3)
#define LED_AUS	PORTC &= ~(1<<PC3);					//LED ausschalten




uint8_t test1, test1_alt, aaa,bbb,ccc, ddd, error;
uint16_t zaehler, test2, test2_alt, test3;
int32_t temperatur;
uint32_t pressure, humidity;
uint8_t sekunden, sekunden_alt, minuten, stunden;
volatile uint8_t 	received_byte;
int32_t temp, press, hum;
uint16_t vork, nachk; 
uint8_t flag_send, flag_send_index;
// String für Zahlenausgabe
char string[40] = "";
uint8_t ms, ms10,ms100,sec,min,entprell, state;
uint8_t end_ms100, end_sec, end_min;
enum state{WAIT, COUNT, TIME, TIME_WAIT,FLIGHT_TIME};
uint8_t str_len;

uint8_t test=0;

static uint8_t read_line(char* buffer, uint8_t buffer_length);
static uint32_t strtolong(const char* str);
static uint8_t find_file_in_dir(struct fat_fs_struct* fs, struct fat_dir_struct* dd, const char* name, struct fat_dir_entry_struct* dir_entry);
static struct fat_file_struct* open_file_in_dir(struct fat_fs_struct* fs, struct fat_dir_struct* dd, const char* name); 
static uint8_t print_disk_info(const struct fat_fs_struct* fs);


/* Function prototypes */
static void setup(void);

void sd_card(const uint8_t * sddata, uint8_t close)
{
	//when close != 0, sd_card can be closed
	//else sd_card can be opened or be written
	
	static struct partition_struct* partition;
	static struct fat_fs_struct* fs;
	static struct fat_dir_struct* dd;
	static struct fat_file_struct* fd;
	static uint8_t action=0;
	
	#define SD_OPEN  0
	#define SD_WRITE 1
	#define SD_CLOSE 2
	
	if(close)action=SD_CLOSE;//close sd-card
	
	switch(action)
	{
		case SD_OPEN:	// setup sd card slot
						if(!sd_raw_init())
						{
							glcd_draw_string_xy(0,0,"Init fail...");
						}

						// open first partition 
						partition = partition_open(sd_raw_read,sd_raw_read_interval,sd_raw_write,sd_raw_write_interval,0);
						if(!partition)
						{
							// If the partition did not open, assume the storage device
							// is a "superfloppy", i.e. has no MBR.
							partition = partition_open(sd_raw_read,sd_raw_read_interval,sd_raw_write,sd_raw_write_interval,-1);
							if(!partition)
							{
								glcd_draw_string_xy(0,0,"part failed...");
							}
						} 
						 /* open file system */
						fs = fat_open(partition);
						if(!fs)
						{
							glcd_draw_string_xy(0,0,"fs failed...");
						}

						/* open root directory */
						struct fat_dir_entry_struct directory;
						fat_get_dir_entry_of_path(fs, "/", &directory);

						dd = fat_open_dir(fs, &directory);
						if(!dd)
						{
							glcd_draw_string_xy(0,0,"root failed...");
						}

						/* print some card information as a boot message */
						print_disk_info(fs);
						
						fd = open_file_in_dir(fs, dd, "roman.log");
						if(!fd)
						{
							uart_puts("could not open test.txt\n");
						}
						uart_puts("SD is ready...\n");
						action=SD_WRITE;//sd_card is opened, next step is sd_write
						break;
		case SD_WRITE:	/*int32_t file_pos = 0;
						if(!fat_seek_file(fd, &file_pos, FAT_SEEK_CUR))
						{
							uart_puts("file seek errorl\n");
						}*/
						// file_pos now contains the absolute file position
						sprintf(string,"eins");
						str_len=strlen(string);
						fat_write_file(fd,(const uint8_t *)string,str_len);
						sd_raw_sync();
						sprintf(string,"\nzwei");
						str_len=strlen(string);
						fat_write_file(fd,(const uint8_t *)string,str_len);
						sd_raw_sync();
						sprintf(string,"\ndrei vier fünf sech sieben acht");
						str_len=strlen(string);
						fat_write_file(fd,(const uint8_t *)string,str_len);
						sd_raw_sync();
						uart_puts("writing sucessfull\n");
						break;
		case SD_CLOSE:	/* close directory */
						fat_close_dir(dd);

						/* close file system */
						fat_close(fs);

						/* close partition */
						partition_close(partition);
						action=SD_OPEN; //sd_card was closed. Next call is opening again
						uart_puts("SD is closed...\n");
						break;
	}//end of switch
}//end of sd_card()

ISR (TIMER1_COMPA_vect)
{
	ms10++;
	if(entprell != 0)entprell--;
	if(ms10==10)	//10ms
	{
		ms10=0;
		ms100++;
	}
    if(ms100==10)	//100ms
	{
		ms100=0;
		sec++;
	}
	if(sec==60)	//Minute
	{
		sec=0;
		min++;
	}
}

int main(void)
{
	/* Backlight pin PL3, set as output, set high for 100% output */
	DDRB |= (1<<PB2);
	PORTB |= (1<<PB2);
	//PORTB &= ~(1<<PB2);
	DDRC |=(1<<PC3); 	//Ausgang LED
	PORTC |= (1<<PC3);	//Led ein
	DDRD &= ~((1<<PD6) | (1<<PD2) | (1<<PD5)); 	//Taster 1-3
	PORTD |= ((1<<PD6) | (1<<PD2) | (1<<PD5)); 	//PUllups für Taster einschalten
	DDRD &= ~(1<<PD4); //T0 Counter Input
	TCCR0B |= (1<<CS02) | (1<<CS01) | (1<<CS00);//Counter 0 enabled clock on rising edge
	//Timer 1 Configuration
	OCR1A = 0x009C;	//OCR1A = 0x3D08;==1sec
	TCCR1B |= (1 << WGM12);
    // Mode 4, CTC on OCR1A
    TIMSK1 |= (1 << OCIE1A);
    //Set interrupt on compare match
    TCCR1B |= (1 << CS12) | (1 << CS10);
    // set prescaler to 1024 and start the timer
    sei();
    // enable interrupts
	setup();
	
	zaehler=222;
	entprell=0;
	test1=0;
	test1_alt=0;
	test2=0;
	test2_alt=0;
	ms10=0;
	ms100=0;
	sec=0;
	min=0;
	sekunden=0;
	sekunden_alt=0;
	minuten=0;
	stunden=0;
	test3=0;
	aaa=255;
	bbb=0;
	ccc=0;
	ddd=0;
	temperatur=0;
	error=0;
	pressure = 0;
	humidity = 0;
	nachk =0;
	vork =0;
	flag_send=0;
	flag_send_index=0;
	str_len=0;//length of a string

	glcd_tiny_set_font(Font5x7,5,7,32,127);
	glcd_clear_buffer();

	BME280_init();
	BME280_set_filter(BME280_FILTER_16);
	BME280_set_standby(BME280_TSB_10);
	BME280_set_measure(BME280_OVER_16, BME280_HUM);
	BME280_set_measure(BME280_OVER_16,  BME280_PRESS);
	BME280_set_measure(BME280_OVER_16, BME280_TEMP);
	BME280_set_measuremode(BME280_MODE_NORM);

	glcd_clear();
	/* setup uart */
	uart_init();
	
	
	sprintf(string,"string %d",str_len);
	glcd_draw_string_xy(0,0,string);
		while(1) 
		{	

		/*
			BME280_readout(&temp, &press, &hum);
				

			sprintf(string,"Temp: %lu", temp);
			glcd_draw_string_xy(0,0,string);
			
			sprintf(string,"pres %lu",press);
			glcd_draw_string_xy(0,15,string);
				
			sprintf(string,"hum %lu", hum);
			glcd_draw_string_xy(0,30,string);
			*/
			/*
			 * 
			 * "Control characters
				$P or $p= form feed
				$L or $l= line feed
				$R or $r= carriage return
				$T or $t= tabulator
				$N or $n= new line"
			 * */
			 
		/*
					flag_send_index=1;
					uart_send_char('A');
					uart_send_char('\r');
					uart_send_char('\n');
					uart_send_string("Feuchtigkeit");
					uart_send_char(';');
					uart_send_string("Temperatur");
					uart_send_char(';');
					uart_send_string("Luftdruck");
					uart_send_char('\r');
					uart_send_char('\n');
					uart_send_char(';');
					uart_send_string("%");
					uart_send_char(';');
					uart_send_string("C");
					uart_send_char(';');
					uart_send_string("hPa");
					uart_send_char('\r');
					uart_send_char('\n');
				*/
					 
				/*Umwandlung in Vorkomma und Nachkommawerte 
				 * um ueber uart float uebertragen zu können*/
				 /*
				vork = hum/1024;
				nachk = hum-vork*1024;
							
				uart_send_u16data(vork);
				uart_send_char('.');
				uart_send_u16data(nachk);
				uart_send_char(';');
				
				vork = temp /100;
				nachk = temp-vork*100;
				
				uart_send_u16data(vork);
				uart_send_char('.');
				uart_send_u16data(nachk);
				uart_send_char(';');
				
				vork = press/100;
				nachk = press-vork*100;
				
				uart_send_u16data(vork);
				uart_send_char('.');
				uart_send_u16data(nachk);
				
				uart_send_char('\r');
				uart_send_char('\n');
			*/
		if(T_BLUE && (!entprell))
		{
			entprell=RELOAD_ENTPRELL;
			LED_EIN;
			flag_send=0;
			flag_send_index=1;
			test++;
			sprintf(string,"BLUE %d", test);
			glcd_draw_string_xy(0,10,string);
			sd_card((const uint8_t *)string,0);
		}
		if(T_GREEN && (!entprell))
		{
			entprell=RELOAD_ENTPRELL;
			LED_EIN;
			flag_send=0;
			flag_send_index=1;
			test++;
			sprintf(string,"GREEN %d", test);
			glcd_draw_string_xy(0,20,string);
			
		}
		if(T_YELLOW && (!entprell))
		{
			entprell=RELOAD_ENTPRELL;
			flag_send=1;
			LED_AUS;
			flag_send_index=0;
			test++;
			sprintf(string,"YELLOW %d", test);
			glcd_draw_string_xy(0,30,string);
			sd_card((const uint8_t *)string,1);
		}
		
		
		
	
	glcd_write();
	}//End of while
	
	return 0;
}//end of main

uint8_t read_line(char* buffer, uint8_t buffer_length)
{
    memset(buffer, 0, buffer_length);

    uint8_t read_length = 0;
    while(read_length < buffer_length - 1)
    {
        uint8_t c = uart_getc();

        if(c == 0x08 || c == 0x7f)
        {
            if(read_length < 1)
                continue;

            --read_length;
            buffer[read_length] = '\0';

            uart_putc(0x08);
            uart_putc(' ');
            uart_putc(0x08);

            continue;
        }

        uart_putc(c);

        if(c == '\n')
        {
            buffer[read_length] = '\0';
            break;
        }
        else
        {
            buffer[read_length] = c;
            ++read_length;
        }
    }

    return read_length;
}

uint32_t strtolong(const char* str)
{
    uint32_t l = 0;
    while(*str >= '0' && *str <= '9')
        l = l * 10 + (*str++ - '0');

    return l;
}

uint8_t find_file_in_dir(struct fat_fs_struct* fs, struct fat_dir_struct* dd, const char* name, struct fat_dir_entry_struct* dir_entry)
{
    while(fat_read_dir(dd, dir_entry))
    {
        if(strcmp(dir_entry->long_name, name) == 0)
        {
            fat_reset_dir(dd);
            return 1;
        }
    }

    return 0;
}

struct fat_file_struct* open_file_in_dir(struct fat_fs_struct* fs, struct fat_dir_struct* dd, const char* name)
{
    struct fat_dir_entry_struct file_entry;
    if(!find_file_in_dir(fs, dd, name, &file_entry))
        return 0;

    return fat_open_file(fs, &file_entry);
}

uint8_t print_disk_info(const struct fat_fs_struct* fs)
{
    if(!fs)
        return 0;

    struct sd_raw_info disk_info;
    if(!sd_raw_get_info(&disk_info))
        return 0;

    uart_puts_p(PSTR("manuf:  0x")); uart_putc_hex(disk_info.manufacturer); uart_putc('\n');
    uart_puts_p(PSTR("oem:    ")); uart_puts((char*) disk_info.oem); uart_putc('\n');
    uart_puts_p(PSTR("prod:   ")); uart_puts((char*) disk_info.product); uart_putc('\n');
    uart_puts_p(PSTR("rev:    ")); uart_putc_hex(disk_info.revision); uart_putc('\n');
    uart_puts_p(PSTR("serial: 0x")); uart_putdw_hex(disk_info.serial); uart_putc('\n');
    uart_puts_p(PSTR("date:   ")); uart_putw_dec(disk_info.manufacturing_month); uart_putc('/');
                                   uart_putw_dec(disk_info.manufacturing_year); uart_putc('\n');
    uart_puts_p(PSTR("size:   ")); uart_putdw_dec(disk_info.capacity / 1024 / 1024); uart_puts_p(PSTR("MB\n"));
    uart_puts_p(PSTR("copy:   ")); uart_putw_dec(disk_info.flag_copy); uart_putc('\n');
    uart_puts_p(PSTR("wr.pr.: ")); uart_putw_dec(disk_info.flag_write_protect_temp); uart_putc('/');
                                   uart_putw_dec(disk_info.flag_write_protect); uart_putc('\n');
    uart_puts_p(PSTR("format: ")); uart_putw_dec(disk_info.format); uart_putc('\n');
    uart_puts_p(PSTR("free:   ")); uart_putdw_dec(fat_get_fs_free(fs)); uart_putc('/');
                                   uart_putdw_dec(fat_get_fs_size(fs)); uart_putc('\n');

    return 1;
}


void get_datetime(uint16_t* year, uint8_t* month, uint8_t* day, uint8_t* hour, uint8_t* min, uint8_t* sec)
{
    *year = 2007;
    *month = 1;
    *day = 1;
    *hour = 0;
    *min = 0;
    *sec = 0;
}


static void setup(void)
{
	/* Set up glcd, also sets up SPI and relevent GPIO pins */
	glcd_init();
}
