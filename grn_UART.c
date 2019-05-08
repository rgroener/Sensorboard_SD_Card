#include <stdio.h>
#include <avr/io.h>
#include <stdlib.h>


void uart_send_char(char c)
{
	while((UCSR0A & (1<<UDRE0)) == 0){};
    UDR0 = c;
}
void uart_send_string(char *s)
{
	while(*s != 0x00)
	{
		uart_send_char(*s);
		s++;
	}
}//end of send_string

void uart_send_u8data(uint8_t d)
{
	char buffer[10];
	
	itoa(d,buffer,10);
	uart_send_string(buffer);
}
void uart_send_u16data(uint16_t d)
{
	char buffer[10];
	
	utoa(d,buffer,10);
	uart_send_string(buffer);
}
/*
void uart_string_buff_reset(void)
{
	z_string_buffer = string_buffer; //Zeiger auf globbales array
}*/

