#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 4000000ul

const unsigned int tcnt1_init = F_CPU / (64 * 1) - 1;
//const unsigned int tcnt1_init = 62499u;
const unsigned char bit_dict[] = {
  0b00111111, 0b00000110, 0b01011011, 0b01001111,
  0b01100110, 0b01101101, 0b01111101, 0b00100111,
  0b01111111, 0b01101111, 0b01110111, 0b01111100,
  0b00111001, 0b01011110, 0b01111001, 0b01110001
};
const char colon_dict[] = { 0b00000011, 0b00000000 };
const unsigned int poll_thresh     = 0x0200;
const unsigned int long_thresh     = 0x3000;
const unsigned int long_input_wait = 0x0400;

volatile static unsigned char sw_enable = 0b00000111;
//   timer by 4x4 bits in the order of 10m,1m,10s,1s;
volatile static unsigned int timer = 0;

int start_buzzer(void) {
  TCCR2 = (1 << WGM21) | (0 << WGM20) | (1 << COM20) | (3 << CS20);
  OCR2 = 64;
}

int stop_buzzer(void) {
  PORTB &= 0b11110111;
  TCCR2 = 0;
}

void start_short_buzzer(void) {
  TIMSK  |= (1 << TOIE0);
  TCCR0  =  (5 << CS00); // N=1024
  TCNT0  =  0x80;        // N=512
  start_buzzer();
}

void stop_short_buzzer(void) {
  TIMSK &= ~(1 << TOIE0);
  TCCR0  =  (0 << CS00);
  stop_buzzer();
}

void timer1_clear(void) {
  timer  = 0x0000;
  TCCR1A = 0;
  TCCR1B = 0;
  TIMSK  &= ~(1 << OCIE1A);
}

void timer1_decrement(void) {
  if      (timer == 0x0001) { timer1_clear(); start_buzzer(); } // because of post-decrement
  else if (timer &  0x000f) timer -= 0x0001;
  else if (timer &  0x00f0) timer -= 0x0007; //(-0x0010 + 0x000a - 0x0001);
  else if (timer &  0x0f00) timer -= 0x00a7; //(-0x0100 + 0x0060 - 0x0010 + 0x000a - 0x0001);
  else if (timer &  0xf000) timer -= 0x06a7; //(-0x1000 + 0x0a00 - 0x0100 + 0x0060 - 0x0010 + 0x000a - 0x0001);
}

void minute_increment(void) {
  TCNT1H = 0;
  TCNT1L = 0;
  if      ((timer & 0x0f00) < 0x0900) timer += 0x0100;
  else if ((timer & 0xf000) < 0x9000) timer += 0x0700;
}

void second_increment(void) {
  TCNT1H = 0;
  TCNT1L = 0;
  if      ((timer & 0x000f) < 0x0009) timer += 0x0001;
  else if ((timer & 0x00f0) < 0x0050) timer += 0x0007; //(0x0010 - 0x0009)
  else if ((timer & 0x0f00) < 0x0900) timer += 0x00a7; //(0x0100 - 0x0059)
  else if ((timer & 0xf000) < 0x9000) timer += 0x06a7; //(0x1000 - 0x0959)
}

inline unsigned char is_counter_running(void) {
  return (TIMSK & (1 << OCIE1A));
}

inline unsigned char is_buzzer_running(void)  {
  return (TCCR2 & (7 << CS20));
}

void stop_count(void) {
  TIMSK  = 0;
  TCCR1B = 0;
  stop_buzzer();
  sw_enable = 0b00000111;
}

void start_count(void) {
  TIMSK  |= (1 << OCIE1A);
  OCR1AH = (unsigned char) (tcnt1_init >> 8); // reset timer
  OCR1AL = (unsigned char) (tcnt1_init >> 0); // reset timer
  TCCR1B |= (1 << WGM12) | (3 << CS10); // clk/64
  sw_enable = 0b00000001;
}

void polling(void) {
  unsigned char imask;
  unsigned char button_set   = 0x00;
  unsigned char button_clear = 0x00;
  unsigned char PINC_enable = ~PINC & sw_enable;
  unsigned char beep_short_buzzer = 0;
  volatile static unsigned char button_latch = 0;
  // sw[0] and sw[3] are dummy
  volatile static unsigned int sw[] = { 0, 0, 0, 0, 0 };

  for (imask = 1; imask < 8; imask <<= 1) {
    if (PINC_enable & imask) {
      ++sw[imask];
      if ((imask & 0b00000110) && (sw[imask] == long_thresh)) {
        button_set |= imask;
        sw[imask] -= long_input_wait;
      } else if (sw[imask] == poll_thresh) {
        button_set |= imask;
        beep_short_buzzer = 1;
      }
    } else {
      sw[imask] = 0;
      button_clear |= imask;
    }
  }

  // to supress chattering
  button_clear = ~button_clear;
  button_latch |= button_set;
  button_latch &= button_clear;
  button_set   &= button_clear;

  if (button_set & (1 << 2)) minute_increment();
  if (button_set & (1 << 1)) second_increment();
  if (((button_latch | button_set) & (3 << 1)) == (3 << 1)) {
    timer1_clear();
    sw[1 << 1] = poll_thresh;
    sw[2 << 1] = poll_thresh;
  }
  if (button_set & (1 << 0)) {
    if (is_counter_running() || is_buzzer_running()) stop_count();
    else if (timer) start_count();
  }
  if (beep_short_buzzer) start_short_buzzer();
}

ISR(TIMER1_COMPA_vect) {
  timer1_decrement();
}

ISR(TIMER0_OVF_vect) {
  stop_short_buzzer();
}

void port_init(void) {
  DDRB   = 0xff; PORTB  = 0xff;
  DDRC   = 0xf8; PORTC  = 0x07;
  DDRD   = 0xff; PORTD  = 0xff;
}

void write_digit(unsigned char portb, unsigned char portd) {
  PORTD = 0x00;
  PORTB = portb; // cathode
  PORTD = portd; // anode
  polling();
}

int main(void) {
  //volatile unsigned char i;
  volatile unsigned int colon_count = 0;
  port_init();
  sei();
  while(4) {
    write_digit(1 << 0, bit_dict[(timer >> (4 * 0)) & 0x0f]);
    write_digit(1 << 1, bit_dict[(timer >> (4 * 1)) & 0x0f]);
    write_digit(1 << 2, bit_dict[(timer >> (4 * 2)) & 0x0f]);
    write_digit(1 << 4, bit_dict[(timer >> (4 * 3)) & 0x0f]);
    write_digit(1 << 5, colon_dict[((colon_count++) >> 11) & 1]);
  };
}


