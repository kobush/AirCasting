#include <SHT1x.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include <SPI.h>
#include <Ethernet.h>
#include <HttpClient.h>
#include <Cosm.h>

#include <DHT22.h>

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
DeviceAddress tempDeviceAddress;
int numberOfDevices;
#define TEMPERATURE_PRECISION 12

#define DHT22_PIN 9
DHT22 dht(DHT22_PIN);

// MAC address for your Ethernet shield
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

// Your Cosm key to let you upload data
char cosmKey[] = "T5xJiZKMNobBnyLi8a-QiyUO6eqSAKxvdE1KZkcrOEx3UT0g";

// Define the strings for our datastream IDs
const int streams = 6;
char LM35_TempC_Id[] = "LM35_TempC";
char DS18B20_TempC_Id[] = "DS18B20_TempC";
char SHT15_Humidity_Id[] = "SHT15_Humidity";
char SHT15_TempC_Id[] = "SHT15_TempC";
char DHT22_Humidity_Id[] = "DHT22_Humidity";
char DHT22_TempC_Id[] = "DHT22_TempC";
CosmDatastream datastreams[] = {
  CosmDatastream(LM35_TempC_Id, strlen(LM35_TempC_Id), DATASTREAM_FLOAT),
  CosmDatastream(DS18B20_TempC_Id, strlen(DS18B20_TempC_Id), DATASTREAM_FLOAT),
  CosmDatastream(SHT15_TempC_Id, strlen(SHT15_TempC_Id), DATASTREAM_FLOAT),
  CosmDatastream(SHT15_Humidity_Id, strlen(SHT15_Humidity_Id), DATASTREAM_FLOAT),
  CosmDatastream(DHT22_TempC_Id, strlen(DHT22_TempC_Id), DATASTREAM_FLOAT),
  CosmDatastream(DHT22_Humidity_Id, strlen(DHT22_Humidity_Id), DATASTREAM_FLOAT),
};
// Finally, wrap the datastreams into a feed
CosmFeed feed(96858, datastreams, streams /* number of datastreams */);

float valueSums[streams];
int valueCount = 0;

EthernetClient client;
CosmClient cosmclient(client);

unsigned long previousUploadMillis = 0;
unsigned long uploadInterval = 60 * 1000L; // send every minute


void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600); 

  analogReference(INTERNAL);

  // initialize ethernet from DHCP
  while (Ethernet.begin(mac) != 1)
  {
    Serial.println("Error getting IP address via DHCP, trying again...");
    delay(15000);
  }
  
  sensors.begin();
  delay(100);

  // start Dallas sensors
  Serial.print(F("Locating devices... "));
  // Grab a count of devices on the wire
  numberOfDevices = sensors.getDeviceCount();

  Serial.print(F("Found "));
  Serial.print(numberOfDevices, DEC);
  Serial.println(F(" devices."));

  Serial.print(F("Parasite power is: ")); 
  if (sensors.isParasitePowerMode()) Serial.println(F("ON"));
  else Serial.println(F("OFF"));

  // Loop through each device, print out address
  for(int i=0;i<numberOfDevices; i++)
  {
    // Search the wire for address
    if(sensors.getAddress(tempDeviceAddress, i))
    {
      Serial.print(F("Found device "));
      Serial.print(i, DEC);
      Serial.print(F(" with address: "));
      printAddress(tempDeviceAddress);
      Serial.println();
      
      Serial.print(F("Setting resolution to "));
      Serial.println(TEMPERATURE_PRECISION,DEC);

      // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
      sensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);
      delay(100);
      
      Serial.print(F("Resolution actually set to: "));
      Serial.print(sensors.getResolution(tempDeviceAddress), DEC); 
      Serial.println();
    }else{
      Serial.print(F("Found ghost device at "));
      Serial.print(i, DEC);
      Serial.print(F(" but could not detect address. Check power and cabling"));
    }
  }
}

void printAddress(DeviceAddress addr) {
  byte i;
  for( i=0; i < 8; i++) {                         // prefix the printout with 0x
      Serial.print("0x");
      if (addr[i] < 16) {
        Serial.print('0');                        // add a leading '0' if required.
      }
      Serial.print(addr[i], HEX);                 // print the actual value in HEX
      if (i < 7) {
        Serial.print(", ");
      }
    }
}


void loop() {
  unsigned long startMilis = millis();

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
  valueSums[0] += tempC;

  // Dallas DS18B20
  sensors.requestTemperatures(); // Send the command to get temperatures
  tempC = sensors.getTempCByIndex(0);

  /*
  Serial.print("\t DS18B20 tempC = ");      
  Serial.print(tempC);   
  */
  sendPlotData("DS18B20_TempC", tempC);
  valueSums[1] += tempC;

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
  valueSums[2] += tempC;
  valueSums[3] += humidity;

  delay(250);
  DHT22_ERROR_t chk = dht.readData();
  if (chk == DHT_ERROR_NONE)
  {
    tempC = dht.getTemperatureC();
    humidity = dht.getHumidity();
    sendPlotData("DHT22_TempC", tempC);
    sendPlotData("DHT22_Humidity", humidity);
    valueSums[4] += tempC;
    valueSums[5] += humidity;
  }
  else {
    Serial.print(F("DHT22 "));
    switch(chk) {
      case DHT_ERROR_CHECKSUM:
        Serial.println(F("checksum error"));
        Serial.print(dht.getTemperatureC(),DEC);
        Serial.print(" ");
        Serial.println(dht.getHumidity(),DEC);
        break;
      case DHT_BUS_HUNG:
        Serial.println(F("BUS Hung"));
        break;
      case DHT_ERROR_NOT_PRESENT:
        Serial.println(F("Not Present"));
        break;
      case DHT_ERROR_ACK_TOO_LONG:
        Serial.println(F("ACK time out"));
        break;
      case DHT_ERROR_SYNC_TIMEOUT:
        Serial.println(F("Sync Timeout"));
        break;
      case DHT_ERROR_DATA_TIMEOUT:
        Serial.println(F("Data Timeout"));
        break;
      case DHT_ERROR_TOOQUICK:
        Serial.println(F("Polled to quick"));
        break; 
      default:
        Serial.println(F("unknown error"));
        break;
    }
  }
   
  valueCount++;
  
  unsigned long currentMillis = millis();
  if ((currentMillis - previousUploadMillis)  > uploadInterval)
  {
     previousUploadMillis = currentMillis;
     
     for(int i=0; i<streams; i++)
     {
       datastreams[i].setFloat(valueSums[i] / valueCount);
       valueSums[i] = 0.0;
     }
     valueCount = 0;
     
     Serial.println(F("Uploading it to Cosm"));
     int ret = cosmclient.put(feed, cosmKey);
     Serial.print(F("HTTP result "));
     Serial.println(ret);
  }
  
//  sendPlotData("LOOP", millis()- startMilis);
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
