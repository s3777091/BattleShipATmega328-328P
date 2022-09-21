#include <avr/io.h>
#include <avr/interrupt.h>

#ifndef F_CPU
#define F_CPU 16000000UL  // Set 16 MHz clock speed
#endif

#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((( F_CPU / 16) + ( USART_BAUDRATE / 2)) / ( USART_BAUDRATE )) - 1)


//Handle Data 2 dimensional Array
volatile char data[9][9] = {""}; //Data play
volatile char replayData[9][9] = {""}; //Old Data
volatile char ReceivedByte ; //Variable to store the data (1 byte) read from the register


volatile long int ChangeCoordinate = 0; //Change Coordinate of X and Y
volatile long int TotalShot = 0; // Total Shot of Display

volatile long int TOfHit = 0; //Total Hit Checking
volatile long int ShipShunk = 0; //Total Shin Shunk

volatile long int Number = 1; //Number Checking
volatile long int NumberX = 1; //Number coordinate of X
volatile long int NumberY = 1; //Number coordinate of Y

volatile int flag = 0; // Flag for interrupt

int main(void) {

  //Setup the Baud Rate
  UBRR0H = ( BAUD_PRESCALE >> 8); // Load upper 8- bits of the baud rate value into the high byte of the UBRR0H register
  UBRR0L = BAUD_PRESCALE ; // Load lower 8- bits of the baud rate value into the low byte of the UBRR0L register
  //Configure data format for transimission
  UCSR0C = (1 << UCSZ00 ) | (1 << UCSZ01 ); // Use 8- bit character sizes
  UCSR0B = (1 << RXEN0 ) | (1 << TXEN0 ); // Turn on the transmission and reception circuitry
  UCSR0B |= (1 << RXCIE0);

  // OUTPUT
  PCICR |= (1 << 1); //  0x6C PCMSK1 7:0 PCINT14 PCINT13 PCINT12 PCINT11 PCINT10 PCINT9 PCINT8
  PCMSK1 |= (1 << 4);

  TCCR1B |= (1 << CS12) | (1 << CS10); // prescaler 1024
  TCCR1B |= (1 << WGM12); // enable ctc mode

  OCR1A = 15625; // return 0.1s for prescaler

  DDRC &= ~(1 << 1); // set up intput button for up
  DDRC &= ~(1 << 2); // set up intput button for down
  DDRC &= ~(1 << 3); // set up Change Coordinate
  DDRC &= ~(1 << 4); // set up Shooting
  DDRC &= ~(1 << 5); // set up Replay
  DDRC |= (1 << 0); // Set Up Led
  //

  DDRB |= (1 << 0); //SEGMENT A
  DDRB |= (1 << 1); // LeD
  DDRB |= (1 << 2); //B3
  DDRB |= (1 << 3); //B4
  DDRB |= (1 << 4);// D5
  DDRB |= (1 << 5); //Display 5


  DDRD |= (1 << 2); // SEGMENT B // Pin 1
  DDRD |= (1 << 3); // C // Pin 2
  DDRD |= (1 << 4); // D
  DDRD |= (1 << 5); // E
  DDRD |= (1 << 6); // F
  DDRD |= (1 << 7); // G


  //Turn on all Led Seven Segnment
  PORTB |= (1 << 1);
  PORTB |= (1 << 2);
  PORTB |= (1 << 3);
  PORTB |= (1 << 4);
  PORTB |= (1 << 5);

  //Turn on interrupt
  sei();
  while (1) {

    //Display Game View
    Game_Display();

    //Change coordinate Function
    if (!(PINC & (1 << 3))) {
      _delay_ms(100);
      if (!(PINC & (1 << 3))) {
        ChangeCoordinate = !ChangeCoordinate;
      }
    }

    //Change X Change Y
    if (!ChangeCoordinate == 0) {
      ClickChangeX();
      Number = NumberX;
    } else {
      ClickChangeY();
      Number = NumberY;
    }
    //Replay Game
    if (!(PINC & (1 << 5))) {
      _delay_ms(100);
      if (!(PINC & (1 << 5))) {
        ReplayGame();
      }
    }
  }
}

//Function Replay Game
void ReplayGame() {
  ChangeCoordinate = 0;
  TotalShot = 0;
  TOfHit = 0;
  ShipShunk = 0;
  Number = 1;
  NumberX = 1;
  NumberY = 1;
  flag = 0;

  for (int i = 0; i < 9; i++) {
    for (int j = 0; j < 9; j++) {
      data[i][j] = replayData[i][j];
    }
  }


}

//this function will display Game View the digits on 4 digits 7-segments
void Game_Display() {
  OCR1A = 0.1562;
  if (TIFR1 & (1 << OCF1A)) {
    for (int j = 1; j < 6; j++) {
      PORTD = (0b11111111);
      PORTB &= ~(0b11111 << 1);
      PORTB |= (1 << j); //turn off three other led

      if (j == 1) {
        num(ShipShunk);
      }

      if (j == 2) {
        num(TOfHit);
      }

      if (TotalShot > 9) {
        if (j == 3) {
          num(TotalShot / 10);
        }
        if (j == 4) {
          num(TotalShot % 10);
        }
      } else {
        if (j == 3) {
          num(0);
        }
        if (j == 4) {
          num(TotalShot);
        }
      }

      if (j == 5) {
        //num(data[2][2] - '0'); //Testing Data in Seven Segnment.
        num(Number);
      }

      //Win Game
      if (TotalShot > 16 || ShipShunk == 5) {
        OCR1A = 1.562;
        _delay_ms(100);
      } else {
        _delay_ms(2);
      }

      TIFR1 |= (1 << OCF1A); // check flag
    }
  }
}

//Click Change Value Of X
void ClickChangeX() {
  if (!(PINC & (1 << 1))) { // Up
    _delay_ms(100);
    if (!(PINC & (1 << 1))) {
      NumberX += 1;
      if (NumberX == 9) {
        NumberX = 1;
      }
    }
  }

  if (!(PINC & (1 << 2))) { // Down
    _delay_ms(100);
    if (!(PINC & (1 << 2))) {
      NumberX -= 1;
      if (NumberX == 0) {
        NumberX = 8;
      }
    }
  }
}

//Click Change Value Of Y
void ClickChangeY() {
  if (!(PINC & (1 << 1))) { // Up
    _delay_ms(100);
    if (!(PINC & (1 << 1))) {
      NumberY += 1;
      if (NumberY == 9) {
        NumberY = 1;
      }
    }
  }

  if (!(PINC & (1 << 2))) { // Down
    _delay_ms(100);
    if (!(PINC & (1 << 2))) {
      NumberY -= 1;
      if (NumberY == 0) {
        NumberY = 8;
      }
    }
  }
}

// Function that takes in one integer and display them onto the display
void num(int N) {
  PORTB |= (1 << 0);
  switch (N) {
    case 0:
      PORTB ^= (1 << 0);
      PORTD = (0b1000000 << 1);
      break;
    case 1:
      PORTD = (0b1111001 << 1);
      //      PORTB &= ~(1<<0);
      break;
    case 2:
      PORTB ^= (1 << 0);
      PORTD = (0b0100100 << 1);
      break;
    case 3:
      PORTB ^= (1 << 0);
      PORTD = (0b0110000 << 1);
      break;
    case 4:
      PORTD = (0b0011001 << 1);
      break;
    case 5:
      PORTB ^= (1 << 0);
      PORTD = (0b0010010 << 1);
      break;
    case 6:
      PORTB ^= (1 << 0);
      PORTD = (0b0000010 << 1);
      break;
    case 7:
      PORTB ^= (1 << 0);
      PORTD = (0b1111000 << 1);
      break;
    case 8:
      PORTB ^= (1 << 0);
      PORTD = (0b0000000 << 1);
      break;
    case 9:
      PORTB ^= (1 << 0);
      PORTD = (0b0010000 << 1);
      break;
  }
}

void TurnOnLed() {
  PORTC ^= (1 << 0);
  _delay_ms(500);
  PORTC &= ~(1 << 0);
  _delay_ms(500);
  PORTC ^= (1 << 0);
  _delay_ms(500);
  PORTC &= ~(1 << 0);
  _delay_ms(500);
  PORTC ^= (1 << 0);
  _delay_ms(500);
  PORTC &= ~(1 << 0);
}


//ISR For  Shooting Button
ISR (PCINT1_vect) {
  if (!(PINC & (1 << 4))) {
    _delay_ms(100);
    flag = 1;
    TotalShot += 1;
    //Loop for all data
    for (int nr = 0; nr < 8; nr++) {
      for (int nc = 0; nc < 8; nc++) {
        if (data[NumberX][NumberY + 1] - '0' == 1) {
          if (TOfHit < 10) {
            TOfHit = TOfHit + 1;
            ShipShunk = trunc(TOfHit / 2);
          } else {
            TOfHit = 9;
          }
          data[NumberX][NumberY + 1] = 0;
          TurnOnLed();
        }
      }
    }
  } else {
    PORTC &= ~(1 << 0);
  }
}

//Add Data Interrupt
int charRow = 1;
int charCol = 1;
ISR(USART_RX_vect) {
  char ReceivedByte; //Variable to store the data (1 byte) read from the register
  ReceivedByte = UDR0; // Read the received byte value

  //Remove Space
  if (ReceivedByte == ' ') {
    return;
  }
  //Remove end line and add Char For row Col for Col
  if (ReceivedByte == '\n') {
    charRow = 1;
    charCol++;
    return;
  }
  charRow++;

  data[charCol][charRow] = ReceivedByte;
  replayData[charCol][charRow] = ReceivedByte;
  UDR0 = data[charCol][charRow];
}
