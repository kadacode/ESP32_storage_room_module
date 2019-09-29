//Module based on DS18B20, DHT22, BME280 and SCD30 sensors which uploads temperature, humidity, dewpoint, air pressure and air CO2 content data to two cloud platforms,
//receives and locally stores control variables from mydevices by Cayenne and includes restart in case of software or task hang for long-time autonomy.
//Install the following libraries: Adafruit_Unified_Sensor, CayenneMQTT, DallasTemperature, DHT_sensor_library, OneWire, scd30-master, SparkFun_BME280, SparkFun_SCD30_Arduino_Library

#include <WiFi.h>               //wifi upload
#include <OneWire.h>            //DS18B20, needs a 4.7k ohm resistor data pin-GND. If long cables, power directly from 5V source and use voltage divider to convert level on data wire from 5V to 3.3V. In this code each sensor has its pin so problems due to star/hub configuration of the sensors are avoided. 
#include <DallasTemperature.h>  //If still unstable, recommended to add 0,1k resistor in series on both ends of data line + 100 nF - 10microF capacitor VCC-GND at sensor end. See resources. https://forum.arduino.cc/index.php?topic=20574.0 https://forum.arduino.cc/index.php?topic=514536.0) https://thecavepearlproject.org/2015/03/01/using-ds18b20-sensors-to-make-a-diy-thermistor-string-pt-1-the-build/ https://learn.openenergymonitor.org/electricity-monitoring/temperature/DS18B20-temperature-sensing?redirected=true  https://www.raspberrypi.org/forums/viewtopic.php?f=44&t=36163&sid=5cd8973e6238296bbb202262a8d23543&start=25 https://forums.adafruit.com/viewtopic.php?f=8&t=117189
#include <DHT.h>                //DHT22, needs a 10k ohm resistor (not needed for AM2302 version). If long cables, power directly from 5V source and use voltage divider to convert level on data wire from 5V to 3.3V.
#include <paulvha_SCD30.h>      //SCD30          
#include <SparkFunBME280.h>     //BME280 - a small change needs to be made in this file if used together with the SCD30, see https://github.com/paulvha/scd30/blob/master/examples/Example10_BasicReadings_and_BME280_pvh/Example10_BasicReadings_and_BME280_pvh.ino
#include <CayenneMQTTESP32.h>   //Cayenne - module only knows control variables when pushed from mydevices, so they are stored locally before restart: https://community.mydevices.com/t/digital-actuators-relays-do-not-return-to-state-prior-to-arduino-reset/9072
#include <driver/dac.h>         //Variable Frequency Drive control through DAC analog voltage output (GPIO 25, channel 1)
#include <EEPROM.h>             //for local storage of control variables
#include "esp_system.h"         //watchdog timer for stability: if software times out, restart software
#include "esp_task_wdt.h"       //task watchdog timer: if a task times out, reboot

const int oneWireBus = 4;         // GPIO where the DS18B20 is connected to
const int oneWireBus1 = 16;       // separate pin and channel for each DS18B20 if sensors are in star/hub configuration
const int oneWireBus2 = 17;
float tempBU0, tempBU1, tempBU2 = 0;// used to filter -127 values from three DS18B20 sensors
int fanslider;                    // control variable with slider % received by Cayenne
int fanPCT;                       // used to report the working % back to cloud platforms to log usage of a fan
int updateperiod = 30*1000;       // every 20s measurements and update - no use of delay()
int storageperiod = 60*60*1000;   // every hour do a local storage of control variables - ESP32 flash memory has lifespan of 10^5 to 10^6 writes
unsigned long timenow = 0;        // timer for loop cycle instead of delay
unsigned long timeprevious = 0;   // timer variables for local storage
unsigned long timeelapsed;
hw_timer_t *timer = NULL;         // watchdog
int timeoutperiod = 1.5*updateperiod;// 30s before watchdogs are triggered

#define LED 2                    //LED setup
#define DHTPIN 18                //DHT22 2 sensors on different pins. Not 12 because of https://github.com/espressif/esptool/issues/305
#define DHTPIN1 19 
#define I2CADDR 0x76             //BME280 I2C address
#define relaypin 32              //Not 0 or 1. No ADC2-pins with wifi. Choose from 32-39 https://randomnerdtutorials.com/esp32-pinout-reference-gpios/
#define CAYENNE_PRINT Serial     //Cayenne

OneWire oneWire(oneWireBus);     //each DS18B20 sensor has its own bus for stability with long cables. If short cables, use the simpler method of multiple sensors on one pin.
DallasTemperature sensors(&oneWire);
OneWire oneWire1(oneWireBus1);
DallasTemperature sensors1(&oneWire1);
OneWire oneWire2(oneWireBus2);
DallasTemperature sensors2(&oneWire2);
DHT dht(DHTPIN, DHT22);
DHT dht1(DHTPIN1, DHT22);
SCD30 airSensor;
BME280 mySensor;

const char *ssid =  "i";        // replace with your wifi ssid and wpa2 key
const char *pass =  "p";
//const char *ssid =  "i";       // replace with your wifi ssid and wpa2 key
//const char *pass =  "p";
String apiKey = "p";           //  Enter your Write API key from ThingSpeak
const char* server = "api.thingspeak.com";    //"184.106.153.149" or api.thingspeak.com
char username[] = "u";          // Cayenne authentication info. This should be obtained from the Cayenne Dashboard.
char password[] = "p";
char clientID[] = "c";
WiFiClient client;

void IRAM_ATTR resetModule(){           //watchdog function what to do when timeout - first save control variables in flash memory, then restart
    digitalWrite(LED,LOW);
    EEPROM.write(0, fanslider);
    EEPROM.write(1, digitalRead(relaypin));
    EEPROM.commit();
    ESP.restart();
}

void setup() {

  esp_task_wdt_init(timeoutperiod/1000, true);       //initiate task watchdog, time is in seconds here. times out  'true' so the module reboots if times out 
  esp_task_wdt_add(NULL);                            //task watchdog monitors current task so if setup hangs, reboot

  pinMode(LED,OUTPUT);                  //LED setup
  pinMode(relaypin, OUTPUT);
  EEPROM.begin(2);                      //size is 2 bytes for 2 control variables (0-255)
  int fanslider = EEPROM.read(0);       //start with last known slider % stored in address 0 
  dac_output_voltage(DAC_CHANNEL_1, fanslider*2.55); //slider outputs 0-100, times 2.55 it uses the 0-255 DAC range (close to 0.1-3.2V)
  digitalWrite(relaypin, EEPROM.read(1)); //start with last known relay state stored in address 1
  Serial.begin(115200);                 // Start the Serial Monitor
  sensors.begin();                      // Start the DS18B20 sensors  
  sensors1.begin();
  sensors2.begin();
  dht.begin();                          // DHT22
  dht1.begin();
  mySensor.setI2CAddress(I2CADDR);      //BME280
  mySensor.beginI2C();
  Wire.begin();                         //SCD30
  airSensor.begin(Wire);                //SCD30 This will cause SCD30 readings to occur every two seconds
  delay(10000);                         //for first measurements without zero values (SCD30)
  
  Cayenne.begin(username, password, clientID, ssid, pass);      //cayenne connect
  Serial.println("Connecting to network.");   //thingspeak connect to wifi
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) 
    {   delay(500);
        Serial.print("."); }
  Serial.println("");
  Serial.println("WiFi connected");

  esp_task_wdt_delete(NULL);                          //task watchdog stops monitoring, setup succesful
  
  timer = timerBegin(0, 80, true);                    //watchdog timer setup: timer 0, div 80
  timerAttachInterrupt(timer, &resetModule, true);
  timerAlarmWrite(timer, timeoutperiod*1000, false);  //time is in microseconds here, give it enough time to run through setup() and updateperiod of loop()
  timerAlarmEnable(timer); //enable interrupt
}

void loop() {
  
  timerWrite(timer, 0);                 //reset watchdog timer (feed watchdog)
  esp_task_wdt_add(NULL);               //task watchdog monitors the current task
  esp_task_wdt_reset();                 //reset task watchdog every loop cycle
  timenow = millis();                   //start timer for loop cycle and storage
  timeelapsed = timenow - timeprevious; 
   if (timeelapsed >= storageperiod) {   //do local storage if elapsed time reaches period for storage
    timeprevious = timeprevious + storageperiod;
    EEPROM.write(0, fanslider);
    EEPROM.write(1, digitalRead(relaypin));
    EEPROM.commit();
    }
    
   if (digitalRead(relaypin) == LOW){    //update usage fanPCT to report to Cayenne
    fanPCT = fanslider;
    }
   else {
    fanPCT = 0;
    }
  
  digitalWrite(LED,HIGH);               //LED on, after measurements and data upload LED off

  sensors.requestTemperatures();                  //DS18B20
  float tempDS0 = sensors.getTempCByIndex(0); //measure sensor 0
   while ((isnan(tempDS0) || tempDS0 == -127) && millis() < timenow + 3000) {
    delay(1000);    
    sensors.requestTemperatures();            //
    tempDS0 = sensors.getTempCByIndex(0);
    }
   if (tempDS0 == -127) {                     //filter -127 values with backup previous T
    tempDS0 = tempBU0;
    }
  tempBU0 = tempDS0;
    Serial.print(tempDS0);
    Serial.println("ºC sensor 0");

  sensors1.requestTemperatures();                  //DS18B20  
  float tempDS1 = sensors1.getTempCByIndex(0); //measure sensor 1
   while ((isnan(tempDS1) || tempDS1 == -127) && millis() < timenow + 6000) {
    delay(1000);    
    sensors1.requestTemperatures();            //
    tempDS1 = sensors1.getTempCByIndex(0);
    }
   if (tempDS1 == -127) {                     //filter -127 values with backup previous T
    tempDS1 = tempBU1;
    }
  tempBU1 = tempDS1;
    Serial.print(tempDS1);
    Serial.println("ºC sensor 1");

  sensors2.requestTemperatures();                  //DS18B20    
  float tempDS2 = sensors2.getTempCByIndex(0); //measure sensor 2
   while ((isnan(tempDS2) || tempDS2 == -127) && millis() < timenow + 9000) {
    delay(1000);    
    sensors2.requestTemperatures();            //
    tempDS2 = sensors2.getTempCByIndex(0);
    }
   if (tempDS2 == -127) {                     //filter -127 values with backup previous T
    tempDS2 = tempBU2;
    }
  tempBU2 = tempDS2;
    Serial.print(tempDS2);
    Serial.println("ºC sensor 2");

  float tempDHT0 = dht.readTemperature();     //DHT22 sensor 0
  float humDHT0 = dht.readHumidity(); 
   while ((isnan(humDHT0) || isnan(tempDHT0)) && millis() < timenow + 12000) {
    Serial.println(F("Failed to read from DHT sensor 0!"));
    delay(1000);                               //poging lange kabel 0.1s
    tempDHT0 = dht.readTemperature();   
    humDHT0 = dht.readHumidity(); 
    }
    Serial.print(tempDHT0);
    Serial.println("ºC DHT-sensor 0");
    Serial.print(humDHT0);
    Serial.println("%RV DHT-sensor 0");
                   
  float tempDHT1 = dht1.readTemperature();    //DHT22 sensor 1
  float humDHT1 = dht1.readHumidity(); 
   while ((isnan(humDHT1) || isnan(tempDHT1)) && millis() < timenow + 15000) {
    Serial.println(F("Failed to read from DHT sensor 1!"));
    delay(1000);                               //poging lange kabel 0.1s
    float tempDHT1 = dht1.readTemperature();   
    float humDHT1 = dht1.readHumidity(); 
    }
    Serial.print(tempDHT1);
    Serial.println("ºC DHT-sensor 1");
    Serial.print(humDHT1);
    Serial.println("%RV DHT-sensor 1");

  float dewpDHT0 = calcDewpoint(humDHT0, tempDHT0); //DHT sensors' difference in dewpoint calculation
  float dewpDHT1 = calcDewpoint(humDHT1, tempDHT1); 
  float diffdewp = dewpDHT1-dewpDHT0;
  
    float CO2 = airSensor.getCO2();               //SCD30
    float tempSCD = airSensor.getTemperature();
    float humSCD = airSensor.getHumidity();
    Serial.print("co2(ppm):");
    Serial.print(CO2);
    Serial.print(" SCD30 temp(C):");
    Serial.print(tempSCD, 1);
    Serial.print(" SCD30 humidity(%):");
    Serial.print(humSCD, 1);
    Serial.println();

    float tempBME = mySensor.readTempC();         //BME280
    float humBME = mySensor.readFloatHumidity();
    float pBME = mySensor.readFloatPressure()/100;

    airSensor.setAmbientPressure(pBME);           //SCD30 adjusts its measurements with pressure input from BME280

    Cayenne.loop();                               //cayenne upload - if connection is lost inside the loop, software hangs: https://community.mydevices.com/t/esp8266-hangs-in-cayenne-loop-when-wifi-connection-is-lost/6401/6
    Cayenne.celsiusWrite(1, tempDS0);             // Write data to Cayenne here. (channel number, value)
    Cayenne.celsiusWrite(2, tempSCD);
    Cayenne.virtualWrite(3, humSCD);  
    Cayenne.virtualWrite(4, CO2);  
    Cayenne.celsiusWrite(5, tempDS1); 
    Cayenne.celsiusWrite(6, tempDS2); 
    Cayenne.celsiusWrite(7, tempDHT0);             //skip 8,9 (used for relay and DAC control)
    Cayenne.celsiusWrite(10, tempDHT1);
    Cayenne.virtualWrite(11, humDHT0);  
    Cayenne.virtualWrite(12, humDHT1); 
    Cayenne.celsiusWrite(13, dewpDHT0);
    Cayenne.celsiusWrite(14, dewpDHT1);  
    Cayenne.virtualWrite(15, diffdewp); 
    Cayenne.virtualWrite(16, fanPCT);
    Cayenne.celsiusWrite(17, tempBME); 
    Cayenne.virtualWrite(18, humBME);
    Cayenne.virtualWrite(19, pBME);
    
    Serial.println("%. Send to Cayenne");

  if (client.connect(server,80))                  // thingspeak upload
    {                   
    String postStr = apiKey;
    postStr +="&field1=";
    postStr += String(tempDS0);
    postStr += "&field2=";
    postStr += String(tempSCD);
    postStr += "&field3=";
    postStr += String(humSCD);
    postStr += "&field4=";
    postStr += String(CO2);
    postStr += "&field5=";
    postStr += String(tempDS1);
    postStr += "&field6=";
    postStr += String(tempDS2);
    postStr += "&field7=";
    postStr += String(tempDHT0);
    postStr += "&field8=";
    postStr += String(tempDHT1);
    postStr += "&field9=";
    postStr += String(humDHT0);
    postStr += "&field10=";
    postStr += String(humDHT1);
    postStr += "\r\n\r\n";                    //and so on

    client.print("POST /update HTTP/1.1\n");
    client.print("Host: api.thingspeak.com\n");
    client.print("Connection: close\n");
    client.print("X-THINGSPEAKAPIKEY: "+apiKey+"\n");
    client.print("Content-Type: application/x-www-form-urlencoded\n");
    client.print("Content-Length: ");
    client.print(postStr.length());
    client.print("\n\n");
    client.print(postStr);
                             
    Serial.println("%. Send to Thingspeak");
}
    digitalWrite(LED,LOW);                   //measurements and uploads done
    
    while(millis() < timenow + 18000){        //pause needed for thingspeak because of https://community.thingspeak.com/forum/announcements/sending-data-to-thingspeak-api-from-device-but-channel-is-not-updated-what-changed/
    }            
    client.stop();
   
    Serial.println("Waiting...");
        while(millis() < timenow + updateperiod){        //wait until updateperiod is over
    }
}

  float calcDewpoint(float hum, float temp) { //function for dewpoint calculation of DHT measurements, used in loop()
  float k;
  k = log(hum/100) + (17.62 * temp) / (243.12 + temp);
  return 243.12 * k / (17.62 - k);
}

    CAYENNE_IN(9)                   //Cayenne DAC voltage control, 9 is virtual channel with slider widget
    {
    fanslider = getValue.asInt();
    dac_output_enable(DAC_CHANNEL_1);
    dac_output_voltage(DAC_CHANNEL_1, fanslider*2.55); //slider outputs 0-100, times 2.55 it uses the 0-255 DAC range (close to 0.1-3.2V)
    //possibly improved with PWM pin output and a PWM - 10V DC adapter module, so the incoming voltage in the VFD is higher, lowering noise on the control value compared to 3.3V level https://randomnerdtutorials.com/esp32-pwm-arduino-ide/
 }

    CAYENNE_IN(8)                   //Cayenne relay control (normally open), 8 is virtual channel with button widget https://community.mydevices.com/t/15-second-delay/9145/8
    {
  if (getValue.asInt() == 0) {
    digitalWrite(relaypin, HIGH);
  }
  else {
    digitalWrite(relaypin, LOW);
  }
 }
