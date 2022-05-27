/* HT9200B DTMF Generator Test                             BroHogan 7/17/09
 * SETUP FOR SERIAL MODE - Requires 3 pins - Data and Clock (100KHz) and CE
 * Wire per data sheet - 3.57MHz xtal (4.0MHz won't dial) S/P to GND = serial
 * Serial Mode also provides 8 "pure" tones (N/A w/ parallel)
 */
#include <Arduino.h>
// #include "HT9200.h"                     // defines for HT9200 DTMF chip
#define CLOCK_PIN  D2                    // Clock (serial)
#define DATA_PIN   D3                    // Data (serial)
#define CE_PIN     D4                    // Chip Enable pin (must control)
#define DTMF_OFF 24
#define HZ_697    16
#define HZ_770    17
#define HZ_852    18
#define HZ_941    19
#define HZ_1209   20
#define HZ_1336   21
#define HZ_1477   22
#define HZ_1633   23
#define PHONE_NMBR "11111"         // phone # to dial
char PhoneNum[] = PHONE_NMBR;           // load phone # for dialer




void DTMF_Out (byte digit, long duration, long pause){  // FOR SERIAL COMM
  if (digit == 0) digit = 10;           // take care of 0 here
  for (byte i=0; i<5; i++){
    digitalWrite(CLOCK_PIN, LOW);      // clock high while setting data
    digitalWrite(DATA_PIN, bitRead(digit,i)); // set data LSB->MSB
    delayMicroseconds(5);               // 1/2 of 100K Clock speed
    digitalWrite(CLOCK_PIN, HIGH);       // clock low to latch data
    delayMicroseconds(5);               // 1/2 of 100K Clock speed
  }
  delay(duration);                      // how long tone will play
  if (pause != 0){                      // tone sounds continuously if zero
    for (byte i=0; i<5; i++){           // same as above
      digitalWrite(CLOCK_PIN, LOW);
      digitalWrite(DATA_PIN, bitRead(DTMF_OFF,i));
      delayMicroseconds(5);
      digitalWrite(CLOCK_PIN, HIGH);
      delayMicroseconds(5);
    }
    delay(pause);                       // how long pause between tones
  }
}
void Init_HT9200 (){
  digitalWrite(CE_PIN, HIGH);           // start with chip disabled (else you go nuts)
  digitalWrite(CLOCK_PIN, HIGH);        // start with clock pin high
  digitalWrite(CE_PIN, LOW);            // now enable the chip
  delay(1000);                            // delay 10ms to ramp up the ocillator
  DTMF_Out (DTMF_OFF,1,0);              // turn off ant tone from previous run
}

void Dialer(){  // cycles through all tones
  for (byte i=0; i<5; i++){
    DTMF_Out(PhoneNum[i]-'0',500,100);  // 1/2 sec tome with 1/10 pause
  }
}

void Pure_Tones(){
  DTMF_Out (HZ_697,1000,0);
  DTMF_Out (HZ_770,1000,0);
  DTMF_Out (HZ_852,1000,0);
  DTMF_Out (HZ_941,1000,0);
  DTMF_Out (HZ_1209,1000,0);
  DTMF_Out (HZ_1336,1000,0);
  DTMF_Out (HZ_1477,1000,0);
  DTMF_Out (HZ_1633,1000,0);

  DTMF_Out (DTMF_OFF,1,0);              // turn off DTMF because zero pause used
}



void setup() {
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(CE_PIN, OUTPUT);
  Init_HT9200();                        // init the chip
}

void loop() {
  Dialer();                             // dials phone
  delay(1000);
//   Pure_Tones();                         // plays all the pure tones
//   delay(1000);
} 
