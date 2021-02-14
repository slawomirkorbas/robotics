
#include <avr/interrupt.h>
#include <avr/io.h>

char commandByte = '0';
void setup() {

   // For atmega32u4 microcontroller (Arduino Leonardo board) having clock 16MHz the values for UBRR0 baudrate can be found in AVR baud rate tables:
   // https://cache.amobbs.com/bbs_upload782111/files_22/ourdev_508497.html
   // eg: 9600bps -> 103; 115200bps -> 8
   UBRR1 = 8; // for configuring baud rate of 115200bps
   UCSR1C |= (1 << UCSZ11) | (1 << UCSZ10); // Use 8-bit character sizes
   UCSR1B |= (1 << RXEN1) | (1 << TXEN1) | (1 << RXCIE1);  // Turn on the transmission, reception, and Receive interrupt  
   sei();// enable global interrupt
}

/**
 * Rceive data interrupt handler...
 */
ISR(USART_RX_vect)
{ 
  commandByte = UDR1;// read the received data byte in temp
}

void transmit(byte value) {
  //Wait for empty transmit buffer
  while( !( UCSR1A & (1 << UDRE1)) );
  UDR1 = value;
}

void loop() 
{
    // do some things in this main loop...
    delay(200);
    transmit('x');
 
    
}
