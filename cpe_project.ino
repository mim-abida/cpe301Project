#include <LiquidCrystal.h>
// #include <DHT.h>

//define Port A register pointers
volatile unsigned char* port_a = (unsigned char*) 0x22; 
volatile unsigned char* ddr_a  = (unsigned char*) 0x21; 
volatile unsigned char* pin_a  = (unsigned char*) 0x20; 

//define Port B register pointers
volatile unsigned char* port_b = (unsigned char*) 0x25; 
volatile unsigned char* ddr_b  = (unsigned char*) 0x24; 
volatile unsigned char* pin_b  = (unsigned char*) 0x23; 

//define ADC
volatile unsigned char* my_ADMUX = 0x7C;
volatile unsigned char* my_ADCSRB = 0x7B;
volatile unsigned char* my_ADCSRA = 0x7A;
volatile unsigned int* my_ADC_DATA = 0x78;
unsigned char bits_ADMUX[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};

//define states
const int disable = 1;
const int idle = 2;
const int RUNNING = 3;
const int error = 4;

int adc_id = 0;
int HistoryValue = 0;
char printBuffer[128];

LiquidCrystal lcd(2, 3, 4, 5, 6, 7);

/*enum state {
  disabled = 0;
  idle = 1;
  running = 2;
  error = 3;
};*/

void disabledState(){
    // keeps LCD blank
  lcd.clear();
  lcd.noDisplay();
  
    // turns on yellow LED on pin 8
  *port_b = 0B00100000;

// check for if start button is pressed
}

void idleState(){
// monitor temp and humidity and print them
// if button pressed, turn off monitor
// go into running state when temp above threshold
// send time when it goes into another state
// water level too low = error state
    // green LED
  *port_b = 0B00010000;
}

void runningState(){
// monitor temp and humidity and print them
// if button pressed, turn off monitor
    // blue LED
  *port_b = 0B00001000;
// turn on motor 
// temp too low = idle state 
// water too low = error state 
}

void errorState(){
// monitor temp and humidity and print them
// if button pressed, turn off monitor
    // red LED + error message on LCD
  *port_b = 0B00000100;
// go to idle state when water level is okay
}

void setup(){

  lcd.begin(16, 2);
  
  //adc_init();

  Serial.begin(9600);

  //set PB7 (LED) to OUTPUT
  *ddr_b |= 0x80;
}

void loop() {

   int state = 0;

    switch(state){
      case 0: disabledState();
              break;
      case 1: idleState();
              break;
      case 2: runningState();
              break;
      case 3: errorState();
              break;
      default:
              break;
    }
    
    
    
    //int value = readWater(adc_id); // get adc value

    /*if(((HistoryValue>=value) && ((HistoryValue - value) > 10)) || ((HistoryValue<value) && ((value - HistoryValue) > 10)))
    {
      sprintf(printBuffer,"ADC%d level is %d\n",adc_id, value);
      Serial.print(printBuffer);
      HistoryValue = value;
    }*/
}

void adc_init()
{
  // set up the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 5 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 3 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 2-0 to 0 to set prescaler selection to slow reading
  
  // set up the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11110111; // clear bit 2-0 to 0 to set free running mode
  
  // set up the MUX Register
  *my_ADMUX  &= 0b00000000; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}

unsigned int adc_read(unsigned char adc_channel_num)
{
  // reset the channel and gain bits
  *my_ADMUX &= 0b11100000;
  
  // clear the channel selection bits
  *my_ADCSRB &= 0b11110111;
  
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    
    // set MUX bit 
    *my_ADCSRB |= 0b00001000;
  }
  
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  
  // set bit ?? of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0b01000000;
  
  // wait for the conversion to complete
  while(*my_ADCSRA & 0b000000000);
  
  // return the result in the ADC data register
  return *my_ADC_DATA;
}

int checkWaterLevel(){
  int value = adc_read(adc_id); // get adc value

  if(((HistoryValue>=value) && ((HistoryValue - value) > 10)) || ((HistoryValue<value) && ((value - HistoryValue) > 10))) {
    sprintf(printBuffer,"ADC%d level is %d\n",adc_id, value);
    Serial.print(printBuffer);
    HistoryValue = value;
  }
}

int checktemperature(){

}

int checkHumidity(){

}

void printTempHumidity(){

}

void sendRTCTime(){

}
