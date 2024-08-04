#include <Wire.h>
#include <DHT.h>
#include <DHT_U.h>
#include <AS5600.h>
#include <ArduinoJson.h>
// ----------- voltage variables ----------
#define voltagePin A0

float voltage = 0;

const float R1 = 10000.0;    // Resistance value of R1 in ohms
const float R2 = 45000.0;    // Resistance value of R2 in ohms
// ----------------------------------------
// ------ wind dir sensing variables ------
float windDirAngle = 0;
unsigned long lastReadDirAngle = 0;
// ----------------------------------------
// ------ windspeed sensing variables -----
#define hallSensorPin 13
volatile unsigned short rotations = 0;
volatile unsigned long rotationsContactBounceTime = 0;
// ----------------------------------------
// ----------- Temp Sensing ---------------
#define DHTPIN 12 //D6
#define DHTTYPE DHT22   // DHT 11/22
DHT_Unified dht(DHTPIN, DHTTYPE);
float dhtTemp = 0;
float dhtHum = 0;

unsigned long lastReadTempHumidity = 0;
// ----------------------------------------
// --------------- general ----------------
#define SensorPower D6

AMS_5600 ams5600;
StaticJsonDocument<5000> dataToServer;

unsigned long lastSendData = 0;
const unsigned long sendDataInterval = 28000;

void IRAM_ATTR isr_rotation();
void enterSleep(short sleepMin, bool inSetup = false);
// ----------------------------------

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  dht.begin();
  pinMode(SensorPower, OUTPUT);

  enterSleep(0, true);

  attachInterrupt(digitalPinToInterrupt(hallSensorPin), isr_rotation, FALLING);
}

void loop()
{
    // Serial.println(String(convertRawAngleToDegrees(ams5600.getRawAngle()),DEC));
    readTempAndHumid();
    readWindDir();
    Serial.println(windDirAngle);
    Serial.println(rotations);
    Serial.println(dhtTemp);
    Serial.println(dhtHum);
    delay(500);

    if((millis() - lastSendData) >= sendDataInterval) {
      fillDataForServer();
      serializeJson(dataToServer, Serial);
      Serial.println();
      lastSendData = millis();
    }
}

void fillDataForServer() {
  dataToServer.clear();
  // copyArray(windDirBuff, dataToServer["dd"]);
  // copyArray(windSpeedBuff, dataToServer["wsd"]);
  //dataToServer["rt"] = rotationTime; 
  //rotationTime = 0;
  dataToServer["r"] = rotations; 
  //rotations = 0;
  //dataToServer["p"] = latestPressure; 
  dataToServer["h"] = String(dhtHum, 2); 
  dataToServer["t1"] = String(dhtTemp, 2); 
  //dataToServer["t2"] = String(latestTemp2, 2); 
  //dataToServer["v"] = String(voltage, 2);
  dataToServer["Id"] = "43123546";
}

void readWindDir(){
    if((lastReadDirAngle - millis()) >= 1500){
      float angle = convertRawAngleToDegrees(ams5600.getRawAngle());
      windDirAngle = angle;
    }

    lastReadDirAngle = millis();
}

void readTempAndHumid() {
  if((lastReadTempHumidity - millis()) >= 1500){
    // dhtTemp = dht.readTemperature();
    // dhtHum = dht.readHumidity();
    sensors_event_t event;
    dht.humidity().getEvent(&event);
    dhtHum = event.relative_humidity;
    dht.temperature().getEvent(&event);
    dhtTemp = event.temperature;
  }

  // doc["t2"] = isnan(dhtTemp) ? 0 : dhtTemp;
  // doc["h"] = isnan(dhtHum) ? 0 : dhtHum;

  lastReadTempHumidity = millis();
}

float convertRawAngleToDegrees(word newAngle)
{
  /* Raw data reports 0 - 4095 segments, which is 0.087890625 of a degree */
  float retVal = newAngle * 0.087890625;
  return retVal;
}

void IRAM_ATTR isr_rotation () {
  if ((millis() - rotationsContactBounceTime) > 15 ) { // Debounce the switch contact
    rotations = rotations + 1;
    rotationsContactBounceTime = millis();
  }
}

void saveInRTCMem(uint32_t &value, uint32_t offset ){ // 1 offset = 4 bytes = 32 bits
  if(!ESP.rtcUserMemoryWrite(64 + offset, &value, sizeof(value))) { // start from 64 + because below 64 are system settings
    //Serial.println("RTC write failed!");
    while(1)
      yield();
  }
}

void readfromRTCMem(uint32_t &value, uint32_t offset ){ // 1 offset = 4 bytes = 32 bits
  if(!ESP.rtcUserMemoryRead(64 + offset, &value, sizeof(value))) { // start from 64 + because below 64 are system settings
    //Serial.println("RTC read failed!");
    while(1)
      yield();
  }
}

void enterSleep(short sleepMin, bool inSetup) { // isSetup = false

  bool wakingFromSleep = ESP.getResetReason() == "Deep-Sleep Wake";
  uint32_t hoursSlept = 0;
  uint32_t hoursToSleep = 0;
  if(inSetup){
    if(wakingFromSleep) {
      readfromRTCMem(hoursSlept, 3);
      readfromRTCMem(hoursToSleep, 5);

      hoursSlept++;

      if(hoursSlept >= hoursToSleep){
        hoursSlept = 0;
        hoursToSleep = 0;
        saveInRTCMem(hoursSlept, 3);
        saveInRTCMem(hoursToSleep, 5);
        return;
      }else {
        saveInRTCMem(hoursSlept, 3);
        sleepMin = 60;
      }

    }else{
      saveInRTCMem(hoursSlept, 3);
      saveInRTCMem(hoursToSleep, 5);
      return;
    }
  }

  if(sleepMin >= 60 && !inSetup){
    hoursToSleep = (sleepMin / 60);
    saveInRTCMem(hoursToSleep, 5);
    sleepMin = 60;
  }

  digitalWrite(SensorPower, LOW);
  ESP.deepSleep((sleepMin * 6) *1e7);
}
