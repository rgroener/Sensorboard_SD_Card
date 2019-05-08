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
* 
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





#define T_RED !(PIND & (1<<PD5)) && (entprell == 0)
#define T_BLUE !(PIND & (1<<PD6)) && (entprell == 0)
#define T_GREEN !(PIND & (1<<PD2)) && (entprell == 0)
#define RELOAD_ENTPRELL 80

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
char string[30] = "";

uint8_t ms, ms10,ms100,sec,min,entprell, state;
uint8_t end_ms100, end_sec, end_min;
enum state{WAIT, COUNT, TIME, TIME_WAIT,FLIGHT_TIME};











/* Function prototypes */
static void setup(void);

static void setup(void)
{
	/* Set up glcd, also sets up SPI and relevent GPIO pins */
	glcd_init();
}

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
		
		
		
		while(1) 
		{	
			
	

		
			BME280_readout(&temp, &press, &hum);
				

			sprintf(string,"Temp: %lu", temp);
			glcd_draw_string_xy(0,0,string);
			
			sprintf(string,"pres %lu",press);
			glcd_draw_string_xy(0,15,string);
				
			sprintf(string,"hum %lu", hum);
			glcd_draw_string_xy(0,30,string);
			
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
		
		if(T_RED)
		{
			flag_send=1;
			LED_AUS;
			flag_send_index=0;
		}
		
		if(T_BLUE)
		{
			LED_EIN;
			flag_send=0;
			flag_send_index=1;
		}
		
	
	glcd_write();
	}//End of while
	
	return 0;
}//end of main


