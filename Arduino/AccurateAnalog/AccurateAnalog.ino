#include <SHT1x.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// These constants won't change.  They're used to give names
// to the pins used:
const int analogInPin = A2;  // Analog input pin that the potentiometer is attached to
const int tempPin = A3;  

int sensorValue = 0;        // value read from the pot
int tempValue = 0;

double tempVolts;
double tempC;
double humidity;

#define dataPin 2
#define clockPin 3
SHT1x sht1x(dataPin, clockPin);

#define ONE_WIRE_BUS 8
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

void setup() {
  
  analogReference(INTERNAL);

  // initialize serial communications at 9600 bps:
  Serial.begin(9600); 
  
  sensors.begin();
  sensors.setResolution(12);
}

void loop() {

/*  analogReference(DEFAULT);
  double Vcc =  readVcc() / 1000.0;

  // read the analog in value:
  sensorValue = analogRead(analogInPin);            

  double volt = (sensorValue / 1023.0) * Vcc;


  // print the results to the serial monitor:
  Serial.print("VCC = ");      
  Serial.print(Vcc);   

  Serial.print("\t sensor = " );                       
  Serial.print(sensorValue);      
  Serial.print("\t volts = ");      
  Serial.print(volt);   
*/
  // LM35
  double Vcc = 1.1;
  tempValue = analogRead(tempPin);
  tempVolts = (tempValue / 1023.0) * Vcc;
  tempC = tempVolts * 100;
  
  /*
  Serial.print("\t LM35  tempValue = ");
  Serial.print(tempValue);   
  Serial.print("\t tempV = ");
  Serial.print(tempVolts);   
  Serial.print("\t tempC = ");      
  Serial.print(tempC);   
  */
  sendPlotData("LM35_TempC", tempC);

  // Dallas DS18B20
  sensors.requestTemperatures(); // Send the command to get temperatures
  tempC = sensors.getTempCByIndex(0);

  /*
  Serial.print("\t DS18B20 tempC = ");      
  Serial.print(tempC);   
  */
  sendPlotData("DS18B20_TempC", tempC);

  // SHT15
  tempC = sht1x.readTemperatureC();
  humidity = sht1x.readHumidity();

  /*
  Serial.print("\t SHT15 tempC = ");      
  Serial.print(tempC);   
  Serial.print("\t humidity = ");      
  Serial.println(humidity);   
  */
  sendPlotData("SHT15_TempC", tempC);
  sendPlotData("SHT15_Humidity", humidity);
  
  // wait 2 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  delay(2);                     
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

//  long scale_constant = 1125300L;
  long scale_constant = 1106410L; 
  result =  scale_constant / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}
