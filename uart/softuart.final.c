//#define FOSC 1843200// Clock Speed
#define FOSC 1000000// Clock Speed
#define BAUD 2400
#define MYUBRR FOSC/16/BAUD-1
#include <avr/io.h>
#include <util/delay.h>
#define p(X) PORTB=(X);while(4);

void wait(unsigned char cycle) {
  // cycle = 3 * clk < 256
  asm volatile(
    "mov r26, %0\n\t"
    "subi r26, 1\n\t"
    "brne .-4\n\t"
    ::"r"(cycle):"r26"
  );
}

unsigned char soft_USART_Receive(void) {
  unsigned char i, c;
  while (PIND & (1 << 2));
  wait(FOSC / BAUD / 3 + FOSC / BAUD / 6);
  for (i = 0; i < 8; i++) {
    c = (c >> 1) | ((PIND << 5) & 0b10000000u);
    wait(FOSC / BAUD / 3);
  }
  if (! ((PIND << 5) & 0b10000000u)) c = 0;
  wait(FOSC / BAUD / 6);
  return c;
}

void USART_Transmit(unsigned char data) {
  /* Wait for empty transmit buffer */
  while (!(UCSRA & (1 << UDRE)));
  /* Put data into buffer, sends the data */
  UDR = data;
}

unsigned char USART_Receive(void) {
  /* Wait for data to be received */
  while (!(UCSRA & (1 << RXC)));
  /* Get and return received data from buffer */
  return UDR;
}

void USART_Init(unsigned int ubrr) {
  unsigned int ucsrc;
  /* Set baud rate */
  UBRRH = (unsigned char)(ubrr >> 8);
  UBRRL = (unsigned char)ubrr;
  /* Enable receiver and transmitter */
  UCSRB = (1 << RXEN) | (1 << TXEN);
  /* Set frame format: 8data */
  UCSRC = (1 << URSEL) | (1 << USBS) | (3 << UCSZ0);
}

int main(void) {
  unsigned char c;
  DDRB  = 0xff; PORTB = 0x00;
  DDRC  = 0xff; PORTC = 0xff;
  DDRD  = 0xf0; PORTD = 0xff;
  USART_Init(MYUBRR);
  while (4) {
    c = soft_USART_Receive();
    c = USART_Receive();
    PORTB = c;
    USART_Transmit(c);
  }
}


