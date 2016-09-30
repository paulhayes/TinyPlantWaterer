 //clear bit macro
 #ifndef cbi
 #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
 #endif

 //set bit macro
 #ifndef sbi
 #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
 #endif

#include <avr/sleep.h> //Needed for sleep_mode
#include <avr/wdt.h> //Needed to enable/disable watch dog timer
#include "SSD1306_minimal.h"
#include "TinyWireM.h"
#include <avr/power.h>
#include <EEPROM.h>

const byte screenPwr = PB4;
const byte sensorPwr = PB4;
const byte waterSensor = 3;
const byte button = PB1;

const char waterMsg[] = "Press to water";
const char thresholdMsg[] = "Press to \n set threshold value";

int sensorValue = 0;
int thresholdValue = 0;
volatile char buttonWake = 0;

long WDTCRTemp;
long MCUSRTemp;
//This runs each time the watch dog wakes us up from sleep
ISR(WDT_vect) {
  //Don't do anything. This is just here so that we wake up.
}

ISR(PCINT0_vect)
{
    buttonWake = 1;             // Increment volatile variable
}

SSD1306_Mini oled;

void setup()
{
  //clock_prescale_set(clock_div_1);
  EEPROM.get(0,thresholdValue);
  if(thresholdValue<=0){
    thresholdValue = 1023;
  }
  pinMode(sensorPwr,INPUT);
  pinMode(screenPwr,INPUT);
  pinMode(button,INPUT);

  screenOn(true);
  bootInfo();
}

void loop()
{
  sleep();

  sensorOn(true);
  sensorValue = analogRead(waterSensor);
  sensorOn(false);
  screenOn(true);

  if( buttonWake ){
    processButtonPress();
  }
  else {
    checkMoisture();
    screenOn(false);
  }



}

void checkMoisture(){

  if( sensorValue > thresholdValue ){
    screenOn(true);
    printWatering();
    delayOrButton(3000);
    screenOn(false);
    pump();

  }

}

bool delayOrButton(int duration){
  unsigned int elapsed = 0;
  while( elapsed < duration ){
    if( isButtonDown() ){
      return true;
    }
    _delay_ms(50);
    elapsed += 50;
  }
  return false;
}

void processButtonPress(){
  printInfo();
  unsigned int elapsed = 0;

  while( delayOrButton(500) );

  printMessage(thresholdMsg);
  if( delayOrButton(2000) ){
    thresholdValue = sensorValue;
    EEPROM.put(0,thresholdValue);
    printInfo();
  }
  printMessage("");
  while( delayOrButton(500) );
  printMessage(waterMsg);
  while( delayOrButton(2000) ){
    pump();
  }

}

void sensorOn(bool on){
  if(!on){
    digitalWrite(sensorPwr, LOW);
  }
  pinMode(sensorPwr,on ? OUTPUT : INPUT);
  pinMode(waterSensor,INPUT);
  digitalWrite(sensorPwr, on ? HIGH : LOW);
  if( on ){
    _delay_ms(300);
  }
}

void screenOn(bool on){
  if(!on){
    digitalWrite(screenPwr, LOW);
  }
  pinMode(screenPwr,on ? OUTPUT : INPUT);
  if( on ){
    digitalWrite(screenPwr, HIGH);
    _delay_ms(100);
    oled.init(0x78);
    oled.startScreen();

  }
}

void pump(){
  pinMode(button,OUTPUT);
  digitalWrite(button,HIGH);
  _delay_ms(5000);
  digitalWrite(button,LOW);
  pinMode(button,INPUT);
}

void bootInfo(){
  char num[24];
  int len = 0;

  oled.init(0x78);
  oled.startScreen();
  oled.clear();
  oled.cursorTo(0,0);
  len = sprintf (num, "Tiny Plant Waterer");
  oled.printString(num, len);
  oled.cursorTo(0,1);
  oled.printString((char*)"by Paul Hayes");

}

void printWatering(){
  char num[24];
  int len = 0;
  printInfo();
  oled.cursorTo(0,2);
  len = sprintf (num, "Watering!");
  oled.printString(num, len);

}

void printInfo(){
  char num[24];
  int len = 0;

  oled.clear();
  oled.cursorTo(0,0);
  len = sprintf (num, "Sensor    %03i", sensorValue);
  oled.printString(num, len);
  oled.cursorTo(0,1);
  len = sprintf (num, "Threshold %03i", thresholdValue);
  oled.printString(num, len);

}

void printMessage(const char msg[]){
  oled.cursorTo(0,2);
  oled.printString((char*)msg);
}

bool isButtonDown(){
  return digitalRead(button) == HIGH;
}


void sleep(){
  //Power down various bits of hardware to lower power usage
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); //Power down everything, wake up from WDT
  sleep_enable();


  buttonWake = 0;
  sbi(GIMSK,PCIE); // Turn on Pin Change interrupts (Tell Attiny85 we want to use pin change interrupts (can be any pin))
  sbi(PCMSK,PCINT1); //Define which pins are doing the interrupting (digital pin 1, DIP 7 in this example)
  sei();

  ADCSRA &= ~(1<<ADEN); //Disable ADC, saves ~230uA
  setup_watchdog(8); //Setup watchdog to go off after 1sec
  sleep_mode(); //Go to sleep! Wake up 1sec later and check water

  //Check for water
  ADCSRA |= (1<<ADEN); //Enable ADC
  cli();
  cbi(GIMSK,PCIE); // Turn off Pin Change interrupts (Tell Attiny85 we want to use pin change interrupts (can be any pin))
  cbi(PCMSK,PCINT1); //Define which pins are doing the interrupting (digital pin 1, DIP 7 in this example)


  //undoWatchdog();
  //clock_prescale_set(clock_div_1);
}

//Sets the watchdog timer to wake us up, but not reset
//0=16ms, 1=32ms, 2=64ms, 3=128ms, 4=250ms, 5=500ms
//6=1sec, 7=2sec, 8=4sec, 9=8sec
//From: http://interface.khm.de/index.php/lab/experiments/sleep_watchdog_battery/
void setup_watchdog(int timerPrescaler) {

  if (timerPrescaler > 9 ) timerPrescaler = 9; //Limit incoming amount to legal settings

  byte bb = timerPrescaler & 7;
  if (timerPrescaler > 7) bb |= (1<<5); //Set the special 5th bit if necessary

  WDTCRTemp = WDTCR;
  MCUSRTemp  = MCUCR;
  //This order of commands is important and cannot be combined
  MCUSR &= ~(1<<WDRF); //Clear the watch dog reset
  WDTCR |= (1<<WDCE) | (1<<WDE); //Set WD_change enable, set WD enable
  WDTCR = bb; //Set new watchdog timeout value
  WDTCR |= _BV(WDIE); //Set the interrupt enable, this will keep unit from resetting after each int
}


void undoWatchdog(){
  WDTCR = WDTCRTemp;
  MCUCR = MCUSRTemp;
}
