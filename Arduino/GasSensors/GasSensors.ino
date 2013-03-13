#include <SoftwareSerial.h> //Header for software serial communication
SoftwareSerial btSerial(2, 3); //Assign 2 as Rx and 3 as Tx 

#define VBAT_PIN A0

#define CO_HEATER_LO 4
#define CO_HEATER_HI 5
#define CO_SENSOR A1

#define NO2_HEATER 6
#define NO2_SENSOR A2

void setup() 
{
  Serial.begin(9600);
  btSerial.begin(9600);
  
  pinMode(CO_HEATER_LO, OUTPUT);
  pinMode(CO_HEATER_HI, OUTPUT);
  pinMode(NO2_HEATER, OUTPUT);
  
  // wait 1s to powerUp
  delay(1000);
}

void loop() 
{
  updateVBat();
  updateCO();
  updateNO2();
    
  delay(10);
}

unsigned long vbat_update;
const int vbat_interval = 5000;

void updateVBat()
{
  unsigned long current_millis = millis();
  if (current_millis - vbat_update >= vbat_interval)
  {
    vbat_update = current_millis;
    
    float vcc = readVcc() / 1000.0;
    sendPlotData("VCC", vcc);
    
    float vbat = analogRead(VBAT_PIN) * vcc / 1023.0 * 2;
    sendPlotData("VBat", vbat);
  }
  
}

unsigned long co_start; // cycle start
boolean co_measured = false;
float co_load = 97.400; // 100K // load resistor in KOhms

void updateCO() {
  unsigned long current_millis = millis();
  if (current_millis - co_start >= 15000)
  {
    Serial.println(current_millis - co_start);
    co_start = current_millis;
    co_measured = false;
  }
  
  if (current_millis - co_start < 5000)
  {
    // turn 2.35V for 5s
    digitalWrite(CO_HEATER_LO, HIGH);
    digitalWrite(CO_HEATER_HI, LOW);
  }
  else if (current_millis - co_start < 15000)
  {
    // then turn 0.5V for 10s
    digitalWrite(CO_HEATER_LO, LOW);
    digitalWrite(CO_HEATER_HI, HIGH);
  
    if (current_millis - co_start >= 14000 && !co_measured)
    {
      Serial.println(current_millis - co_start);
    
      int co_raw = averageAnalogRead(CO_SENSOR, 5);
      float co_v = co_raw  / 1023.0;
      //float vcc = readVcc() / 1000.0;
      //float co_volts = co_v * vcc;
      float co_res = co_load / (1 - co_v) * co_v;
      sendPlotData("CO_IN", co_v);
      sendPlotData("CO_RS", co_res);
      
      btSerial.print((int)((1-co_v)*100.0));
      btSerial.println(F(";InsertSensorPackageName;MiCS-5525;CO Gas;CO2;response indicator;RI;0;25;50;75;100"));
      btSerial.print(co_res,0);
      btSerial.println(F(";InsertSensorPackageName;MiCS-5525 Res;CO Gas;CO2;resistance;RS;10000;8000;5000;4000;2000"));

      co_measured = true;
    }
  }
}


unsigned long no_start; // cylce start time
float no_load = 2.2 + 22 + 220; // load resistor (KOhms)
//float no_load = 2.2; // 2.2KOhm

/*
  Reads from MICS-2710 NO2 sensor
*/
void updateNO2()
{
  // keep always on
  digitalWrite(NO2_HEATER, LOW);
 
  unsigned long current_millis = millis();
  if (current_millis - no_start >= 5000)
  {
    Serial.println(current_millis - no_start);
    no_start = current_millis;
    
    // read analog input
    int no_raw = averageAnalogRead(NO2_SENSOR, 5);
    sendPlotData("NO2_IN", no_raw); 

    float no_v = (no_raw  / 1023.0); // input ratio
    float vcc = readVcc() / 1000.0; // VCC voltage
    sendPlotData("NO2_V", no_v * vcc);

    // resistance
//    float no_res = no_load / (1 - no_v) * no_v;
    float no_res = no_load * (1-no_v) / no_v;

    sendPlotData("NO2_RS", no_res);
    
    btSerial.print((int)(no_v*100.0));
    btSerial.println(F(";InsertSensorPackageName;MiCS-2710;N02 Gas;NO2;response indicator;RI;0;25;50;75;100"));
    btSerial.print(no_res,4);
    btSerial.println(F(";InsertSensorPackageName;MiCS-2710 Res;N02 Gas;NO2;resistance;RS;2;4;6;8;10"));
  }
}

int averageAnalogRead(int pin, int num) {
  int ret = 0;
  for (int i=0; i<num; i++) {
    ret += analogRead(pin);
    delay(2);
  }
  return ret / num;
}

void sendPlotData(String seriesName, float data)
{
  Serial.print("{");
  Serial.print(seriesName);
  Serial.print(",T,");
  Serial.print(data);
  Serial.println("}");
}

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
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
 
  long result = (high<<8) | low;

  long scale_constant = 1125300L;
//  long scale_constant = 1106410L; 
  result =  scale_constant / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}
