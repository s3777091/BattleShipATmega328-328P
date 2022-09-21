#ifndef F_CPU
#define F_CPU 16000000UL  // Set 16 MHz clock speed
#endif

#include <avr/io.h>
#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((( F_CPU / 16) + ( USART_BAUDRATE / 2)) / ( USART_BAUDRATE )) - 1)

volatile char data[] = "";
volatile int i = 0;
volatile char ReceivedByte;

int main ( void ) {
  //Setup the Baud Rate
  UBRR0H = ( BAUD_PRESCALE >> 8); // Load upper 8- bits of the baud rate value into the high byte of the UBRR0H register
  UBRR0L = BAUD_PRESCALE ; // Load lower 8- bits of the baud rate value into the low byte of the UBRR0L register
  //Configure data format for transimission
  UCSR0C = (1 << UCSZ00 ) | (1 << UCSZ01 ); // Use 8- bit character sizes
  UCSR0B = (1 << RXEN0 ) | (1 << TXEN0 ); // Turn on the transmission and reception circuitry

  UCSR0B |= (1 << RXCIE0);
  sei();

  while (1){
  }
}


ISR(USART_RX_vect) {
  char ReceivedByte ; //Variable to store the data (1 byte) read from the register
  ReceivedByte = UDR0 ; // Read the received byte value
  
  while (data[i] != 0) { /* To send back string */
    while (!(UCSR0A & (1 << UDRE0))); /* Wait for empty transmit buffer*/
    UDR0 = data[i];            /* Put data into buffer, sends the data */
    i++;                             /* increment counter           */
  }
  i = 0;
  
  UDR0 = ReceivedByte;
}
