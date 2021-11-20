#include <LiquidCrystal.h>
// #include <DHT.h>
#include "wiring_private.h"
#include "pins_arduino.h"

#include <Wire.h>
#include <DS3231.h>

DS3231 clock;
RTCDateTime dt;


#define WATER_LEVEL_THRESHOLD 30; //insert appropriate value here
#define lowThreshold 30;
#define highthreshold 50;


uint8_t analog_reference = DEFAULT;

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
const int disable = 0;
const int idle = 1;
const int RUNNING = 2;
const int error = 3; 

int adc_id = 0;
int HistoryValue = 0;
char printBuffer[128];
int tempPin = 0;

LiquidCrystal lcd(2, 3, 4, 5, 6, 7);

void disabledState(){
    // sends time for beginning of state
  sendTime();
    // keeps LCD blank
  lcd.clear();
  lcd.noDisplay();
  
    // turns on yellow LED on pin 8
  *port_b = 0B00000001;

// check for if start button is pressed
}

void setup(){

  lcd.begin(16, 2);
  
  //adc_init();

  Serial.begin(9600);

    // Initialize DS3231
  clock.begin();

  
    // Manual (YYYY, MM, DD, HH, II, SS
    // might be able to just comment this line out during testing
  clock.setDateTime(2021, 12, 06, 05, 30, 00);
  
  // Send sketch compiling time to Arduino
  clock.setDateTime(__DATE__, __TIME__); 

  //set PB7 to OUTPUT
  *ddr_b |= 0x80;

  // set PB6 (button) to INPUT
  *ddr_b &= (0x01 << 6);
  // Initialize PB6 to low
  *port_b &= ~(0x01 << 6); 
}

void loop() {
      // turn on yellow LED for disabled state
    *port_b = 0B00000010;
      // pin 12
    *pin_b = 0B00100000
  int temperature = 0;
  int humidity = 0;
    
   //TODO: check button press
  while(buttonPressed){
    measure_environment(&temperature, &humidity);

    //TODO: find thresholds from serial monitoring

    //runningState
    while(temperature > highThreshold && checkWaterLevel()){
      //runningState()
      //set LED blue
      *port_b = 0B00000100;
      sendTime();
      measure_environment(&temperature, &humidity);
    }

    //idleState
    if(temperature < lowThreshold){
      //set LED green
      *port_b = 0B00010000;
      sendTime();
      measure_environment(&temperature, &humidity);
    }

    //errorState
    while(!checkWaterLevel()){
      //set LED red
      *port_b = 0B00001000;
      sendTime();
      //turn off motor
      
      measure_environment(&temperature, &humidity);
    }
  }
}

//I DONT THINK THIS WORKS 
bool buttonPressed(){
  static uint8_t lastBtnState = LOW;
  uint8_t state;
  if(state = HIGH){
    Serial.println("Button pressed!");
  }
  if (state != lastBtnState) {
    lastBtnState = state;
    if (state == HIGH) {
      
  }
  delay(100);
  }
}


void sendTime(){
  dt = clock.getDateTime();

  // For leading zero look to DS3231_dateformat example

  Serial.print("Switch state time: ");
  Serial.print(dt.year);   Serial.print("-");
  Serial.print(dt.month);  Serial.print("-");
  Serial.print(dt.day);    Serial.print(" ");
  Serial.print(dt.hour);   Serial.print(":");
  Serial.print(dt.minute); Serial.print(":");
  Serial.print(dt.second); Serial.println("");

  delay(1000);
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

bool checkWaterLevel(){
  int value = adc_read(adc_id); // get adc value

  if(((HistoryValue>=value) && ((HistoryValue - value) > 10)) || ((HistoryValue<value) && ((value - HistoryValue) > 10))) {
    HistoryValue = value;
  }

  //TODO: define water level threshold

  (value > threshold) ? true : false;
}

static bool measure_environment( float *temperature, float *humidity )
{
  static unsigned long measurement_timestamp = millis( );

  /* Measure once every four seconds. */
  if( millis( ) - measurement_timestamp > 3000ul )
  {
    if( dht_sensor.measure( temperature, humidity ) == true )
    {
      measurement_timestamp = millis( );
      return( true );
    }
  }

    Serial.print("T: ");
    Serial.print(temperature, 1);
    Serial.print("C, H: ");
    Serial.print(humidity, 1);
    Serial.print("%");

  return( false );
}


/*
//copied from ELEGOO lesson 23: thermometer
int checkTemperature(){
  int tempReading = adc_read(tempPin);
  // This is OK
  double tempK = log(10000.0 * ((1024.0 / tempReading - 1)));
  tempK = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * tempK * tempK )) * tempK );       //  Temp Kelvin
  float tempC = tempK - 273.15;            // Convert Kelvin to Celcius
  
  // Display Temperature in C
  //lcd.setCursor(0, 0);
  //lcd.print("Temp         C  ");
 // lcd.setCursor(6, 0);
  // Display Temperature in C
  lcd.print(tempC);
  delay(500);
}

int checkHumidity(){

}

void printTempHumidity(){

}
*/

void sendRTCTime(){

}

void analogReference(uint8_t mode)
{
  // can't actually set the register here because the default setting
  // will connect AVCC and the AREF pin, which would cause a short if
  // there's something connected to AREF.
  analog_reference = mode;
}

/*
int adc_read(uint8_t pin)
{
  uint8_t low, high;

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  if (pin >= 54) pin -= 54; // allow for channel or pin numbers
#elif defined(__AVR_ATmega32U4__)
  if (pin >= 18) pin -= 18; // allow for channel or pin numbers
#elif defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__)
  if (pin >= 24) pin -= 24; // allow for channel or pin numbers
#elif defined(analogPinToChannel) && (defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__))
  pin = analogPinToChannel(pin);
#else
  if (pin >= 14) pin -= 14; // allow for channel or pin numbers
#endif
  
#if defined(__AVR_ATmega32U4__)
  pin = analogPinToChannel(pin);
  ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin >> 3) & 0x01) << MUX5);
#elif defined(ADCSRB) && defined(MUX5)
  // the MUX5 bit of ADCSRB selects whether we're reading from channels
  // 0 to 7 (MUX5 low) or 8 to 15 (MUX5 high).
  ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin >> 3) & 0x01) << MUX5);
#endif
  
  // set the analog reference (high two bits of ADMUX) and select the
  // channel (low 4 bits).  this also sets ADLAR (left-adjust result)
  // to 0 (the default).
#if defined(ADMUX)
  ADMUX = (analog_reference << 6) | (pin & 0x07);
#endif

  // without a delay, we seem to read from the wrong channel
  //delay(1);

#if defined(ADCSRA) && defined(ADCL)
  // start the conversion
  sbi(ADCSRA, ADSC);

  // ADSC is cleared when the conversion finishes
  while (bit_is_set(ADCSRA, ADSC));

  // we have to read ADCL first; doing so locks both ADCL
  // and ADCH until ADCH is read.  reading ADCL second would
  // cause the results of each conversion to be discarded,
  // as ADCL and ADCH would be locked when it completed.
  low  = ADCL;
  high = ADCH;
#else
  // we dont have an ADC, return 0
  low  = 0;
  high = 0;
#endif

  // combine the two bytes
  return (high << 8) | low;
}
*/

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
