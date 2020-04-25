#include "attiny.h"

// constructor
ATTINY::ATTINY(){
  sleepCycles = 1;
  currentCycles = sleepCycles+1;
}

void ATTINY::setSleeptime(int time){
  sleepCycles = time/8;
}

bool ATTINY::checkAction(){
  if(currentCycles >= sleepCycles){
    resetCycles();
    return true;
  }
  return false;
}

void ATTINY::resetCycles(){
    currentCycles = 0;
}

void ATTINY::incrCycles(){
  currentCycles++;
}

// all page references are for this document:
// http://ww1.microchip.com/downloads/en/devicedoc/doc8006.pdf

void ATTINY::gotoSleep(){
  
  //disable ADC
  ADCSRA &= ~(1<<ADEN);
  //unsigned char statOld = ADCSRA;
  //ADCSRA = 0;
  // WD Reset Flag to 0 (p. 45)
  bitClear(MCUSR, WDRF);
  // WD Change Enable (p. 46)
  bitSet(WDTCSR, WDCE);
  // WD Enable (p. 46)
  bitSet(WDTCSR, WDE);
  // create 8 second WD Timer Prescaler
  // 1 0 0 1 means "8 seconds" (p. 47)
  bitSet(WDTCSR, WDP3);
  bitClear(WDTCSR, WDP2);
  bitClear(WDTCSR, WDP1);
  bitSet(WDTCSR, WDP0);
  // WD Interrupt Enable (p. 45)
  bitSet(WDTCSR, WDIE);

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_mode();
  wdt_disable();
  // enable ADC again
//ADMUX = 0b01100011;   //sets 1.1V IRV, sets ADC3 as input channel,
		      //and left adjusts
//ADCSRA = 0b10000011;  //turn on ADC, keep ADC single conversion mode,
                      //and set division factor-8 for 125kHz ADC clock
ADCSRA |=  (1<<ADEN);
  //ADCSRA = statOld;
}

//discussion here: http://forum.arduino.cc/index.php?topic=222847.0
uint16_t ATTINY::getVoltage()
{
  ADMUX = _BV(MUX5) | _BV(MUX0);
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