
#include <attiny.h>
#include "tinySPI.h"
#include "LoRaWAN.h"
#include <TinyWireS.h>
//Change Keys in secconfig.h for your TTN application:
#include "secconfig.h"

ATTINY at =  ATTINY();

#include <Arduino.h>

//ir-led of proximity sensor (invisible)
//#define irled 7
// power in for diode in proximity sensor
#define irdiode 3
//adc pin (connect to output of sensor)
#define irled 2

// define a threshold for your mailbox.
#define THRESHOLD 15

// init RFM95W
#define DIO0 0
#define NSS  1
#define sbi(sfr,bit) (_SFR_BYTE(sfr) |= _BV(bit))

int readVcc();
int readVoltage();
int readADCValue(int adc);
uint16_t getVoltage(int adc);
RFM95 rfm(DIO0,NSS);

LoRaWAN lora = LoRaWAN(rfm);
unsigned int Frame_Counter_Tx = 0x0000;
int sleepTime= 64;

int measure = 0;
int measure1 = 0;
int measure2 = 0;
int measure3 = 0;
int measure4 = 0;


void setup()
{
  at.setSleeptime(sleepTime);
  rfm.init();
  lora.setKeys(NwkSkey, AppSkey, DevAddr);
  analogReference(DEFAULT);
  pinMode(irled, OUTPUT);
//  pinMode(irsens, INPUT);
//  pinMode(7, INPUT);
//  pinMode(8, INPUT);
//  pinMode(9, INPUT);
//  pinMode(10, INPUT);
  pinMode(irdiode, OUTPUT);
  digitalWrite(irdiode,HIGH);
  delay(1000);
  digitalWrite(irdiode,LOW);
  delay(1000);
//    ADMUX |= (1 << REFS0);
//    ADMUX |= (1 << MUX0);   //combined with next line…
//    ADMUX |= (1 << MUX1);   //sets ADC3 as analog input channel
//ADMUX = 0b01100011;

  ADCSRA =
            (1 << ADEN)  |     // Enable ADC
            (1 << ADPS2) |     // set prescaler to 16, bit 2
            (0 << ADPS1) |     // set prescaler to 16, bit 1
            (0 << ADPS0);      // set prescaler to 16, bit 0
			
  ADCSRB =
            //(1 << ADLAR);      // left shift result (for 8-bit values)
	          (0 << ADLAR);      // right shift result (for 10-bit values)

}

void loop()
{
//Serial.println(at.getVoltage());
   if (at.checkAction())
  {
//  ADCSRA |= (1 << ADEN); //enable ADC
//	ADCSRA |= (1 << ADIE); //enable conversion complete interrupt
    uint8_t Data_Length = 0x0F;
    uint8_t Data[Data_Length];
    digitalWrite(irled,HIGH);
    //digitalWrite(irdiode,HIGH);
    delay(2000);
    //measure = analogRead(irsens);
    //ADCSRA |= (1 << ADSC);         // start ADC measurement
//    while (ADCSRA & (1 << ADSC) ); // wait till conversion complete

    //measure1 = readVcc();
    measure1 = getVoltage(1);
    measure2 = at.getVoltage();
    measure3 = 1125300L/(measure1);
    measure4 = 1125300L/(measure2);
    //measure = at.getVoltage();
    delay(500);
    // measure = analogRead(irsens);
    // measure1 = analogRead(7);
    // measure2 = analogRead(8);
    // measure3 = analogRead(9);
    // measure4 = analogRead(10);
    // delay(1000);
    // measure = analogRead(irsens);
    // measure1 = analogRead(7);
    // measure2 = analogRead(8);
    // measure3 = analogRead(9);
    // measure4 = analogRead(10);
    // delay(1000);
    // measure = analogRead(irsens);
    // measure1 = analogRead(7);
    // measure2 = analogRead(8);
    // measure3 = analogRead(9);
    // measure4 = analogRead(10);

    Data[0] = 0x07;
    Data[1] = (measure1 >> 8) & 0xff;
    Data[2] = (measure1 & 0xff);
    Data[3] = 0x08;
    Data[4] = (measure2 >> 8) & 0xff;
    Data[5] = (measure2 & 0xff);
    Data[6] = 0x09;
    Data[7] = (measure3 >> 8) & 0xff;
    Data[8] = (measure3 & 0xff);
    Data[9] = 0x10;
    Data[10] = (measure4 >> 8) & 0xff;
    Data[11] = (measure4 & 0xff);
    Data[12] = 0xAA;
    Data[13] = (measure >> 8) & 0xff;
    Data[14] = (measure & 0xff);
   // Data[12] = 0x11;




//  unsigned int value = checkLetter();
//   Data[0] = 0xff;
//   Data[1] = 0x00;
//    uint16_t value = at.getVoltage();
//    Data[0] = (value >> 8) & 0xff;
//    Data[1] = (value & 0xff);
   digitalWrite(irled,LOW);
   digitalWrite(irdiode,HIGH);
   lora.Send_Data(Data, Data_Length, Frame_Counter_Tx);
   Frame_Counter_Tx++;
   digitalWrite(irdiode,LOW);
}
  at.setSleeptime(sleepTime);
  at.gotoSleep();
}

//interrupt service routine. Incrementing sleep counter
ISR(WDT_vect)
{
  at.incrCycles();
}


unsigned int checkLetter(){
  digitalWrite(irled,HIGH);
  digitalWrite(irdiode,HIGH);
  unsigned int measure = 0x00;
  //for(int i = 0 ; i <3 ; i++){
    delay(1000);
    measure = analogRead(7);
//  }
  //measure = measure/3;
  digitalWrite(irled,LOW);
  digitalWrite(irdiode,LOW);

  // if(measure > THRESHOLD){
  //   return true;
  // }else{
  //   return false;
  // }
  return measure;
}

int readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  int result = (high<<8) | low;

  //result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

int readVoltage(){
  ADCSRA = 0b10000011;  //turn on the ADC, keep ADC single conversion mode
                      //set division factor-8 for 125kHz ADC clock
  ADCSRA |= (1 << ADSC); 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
  delay(1000);                   //delay 1 second
  int result = (high<<8) | low;
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

uint16_t getVoltage(int adc)
{
  switch (adc)
  {
  case 0:
     ADMUX =0;
    break;
  case 1:
    ADMUX = _BV(MUX0);
    break;
  case 2:
    ADMUX = _BV(MUX1);
    break;
  case 3:
    ADMUX = _BV(MUX1) | _BV(MUX0);
  case 4:
    ADMUX = _BV(MUX2);
    break;
  default:
    break;
  }
  delay(2); 
  ADCSRA |= _BV(ADSC); 
  while (bit_is_set(ADCSRA,ADSC)); 
  uint8_t low  = ADCL;
  uint8_t high = ADCH;
  uint16_t result = (high<<8) | low;
  //1125300 = 1.1 x 1023 x 1000
  //result = 1125300/ (result) ; 
  return result;
}

int readADCValue(int adc){
  switch (adc)
  {
  case 0:
     ADMUX =
            (0 << REFS1) |     // Sets ref. voltage to Vcc, bit 1   
            (0 << REFS0) |     // Sets ref. voltage to Vcc, bit 0
            (0 << MUX5)  |     // use ADC1 for input (PA1), MUX bit 5
            (0 << MUX4)  |     // use ADC1 for input (PA1), MUX bit 4
            (0 << MUX3)  |     // use ADC1 for input (PA1), MUX bit 3
            (0 << MUX2)  |     // use ADC1 for input (PA1), MUX bit 2
            (0 << MUX1)  |     // use ADC1 for input (PA1), MUX bit 1
            (0 << MUX0);       // use ADC1 for input (PA1), MUX bit 0
    break;

  case 1:
     ADMUX =
            (0 << REFS1) |     // Sets ref. voltage to Vcc, bit 1   
            (0 << REFS0) |     // Sets ref. voltage to Vcc, bit 0
            (0 << MUX5)  |     // use ADC1 for input (PA1), MUX bit 5
            (0 << MUX4)  |     // use ADC1 for input (PA1), MUX bit 4
            (0 << MUX3)  |     // use ADC1 for input (PA1), MUX bit 3
            (0 << MUX2)  |     // use ADC1 for input (PA1), MUX bit 2
            (0 << MUX1)  |     // use ADC1 for input (PA1), MUX bit 1
            (1 << MUX0);       // use ADC1 for input (PA1), MUX bit 0
    break;
  case 2:
     ADMUX =
            (0 << REFS1) |     // Sets ref. voltage to Vcc, bit 1   
            (0 << REFS0) |     // Sets ref. voltage to Vcc, bit 0
            (0 << MUX5)  |     // use ADC1 for input (PA1), MUX bit 5
            (0 << MUX4)  |     // use ADC1 for input (PA1), MUX bit 4
            (0 << MUX3)  |     // use ADC1 for input (PA1), MUX bit 3
            (0 << MUX2)  |     // use ADC1 for input (PA1), MUX bit 2
            (1 << MUX1)  |     // use ADC1 for input (PA1), MUX bit 1
            (0 << MUX0);       // use ADC1 for input (PA1), MUX bit 0
    break;
  case 3:
     ADMUX =
            (0 << REFS1) |     // Sets ref. voltage to Vcc, bit 1   
            (0 << REFS0) |     // Sets ref. voltage to Vcc, bit 0
            (0 << MUX5)  |     // use ADC1 for input (PA1), MUX bit 5
            (0 << MUX4)  |     // use ADC1 for input (PA1), MUX bit 4
            (0 << MUX3)  |     // use ADC1 for input (PA1), MUX bit 3
            (0 << MUX2)  |     // use ADC1 for input (PA1), MUX bit 2
            (1 << MUX1)  |     // use ADC1 for input (PA1), MUX bit 1
            (1 << MUX0);       // use ADC1 for input (PA1), MUX bit 0
    break;
  case 4:
     ADMUX =
            (0 << REFS1) |     // Sets ref. voltage to Vcc, bit 1   
            (0 << REFS0) |     // Sets ref. voltage to Vcc, bit 0
            (0 << MUX5)  |     // use ADC1 for input (PA1), MUX bit 5
            (0 << MUX4)  |     // use ADC1 for input (PA1), MUX bit 4
            (0 << MUX3)  |     // use ADC1 for input (PA1), MUX bit 3
            (1 << MUX2)  |     // use ADC1 for input (PA1), MUX bit 2
            (0 << MUX1)  |     // use ADC1 for input (PA1), MUX bit 1
            (0 << MUX0);       // use ADC1 for input (PA1), MUX bit 0
    break;
  
  default:
    break;
  }
  delay(2); // Wait for Vref to settle
  ADCSRA |= (1 << ADSC);         // start ADC measurement
  while (ADCSRA & (1 << ADSC) ); 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
  delay(1000);                   //delay 1 second
  int result = (high<<8) | low;
  // int measure =  ADCH; 
  //return measure;
}