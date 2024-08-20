#include <cmath>
#include <Wire.h>
#include <DHT.h>
#include <DHT_U.h>
#include <EEPROM.h>
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
float windDirAngleRaw = 0;
unsigned long lastReadDirAngle = 0;
float dirOffset = 0;

int dirOffsetMemAddress = 0;

const unsigned short windDirBuffSize = 26;
String windDirBuff[windDirBuffSize];
unsigned short windDirBuffCount = 0;
// ----------------------------------------

// ------ windspeed sensing variables -----
#define hallSensorPin 13
volatile unsigned short rotations = 0;
volatile unsigned long rotationsContactBounceTime = 0;
unsigned long rotationTime = 0;
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
#define SensorPower 14 // D5
#define Button 15 // D8

AMS_5600 ams5600;
StaticJsonDocument<5000> dataToServer;
StaticJsonDocument<500> dataFromServer;

unsigned long lastSendData = 0;
const unsigned long sendDataInterval = 28000;
uint32_t serverErrorCount = 0;

bool offsetMode = false;
bool ledTrigger = false;

void IRAM_ATTR isr_rotation();
void enterSleep(short sleepMin, bool inSetup = false);
// ----------------------------------
void setup()
{
  Serial.begin(115200);
  Wire.begin();
  dht.begin();
  EEPROM.begin(200);

  pinMode(SensorPower, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode (Button, INPUT);

  enterSleep(0, true);

  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(SensorPower, HIGH);

  readfromRTCMem(serverErrorCount, 1);

  attachInterrupt(digitalPinToInterrupt(hallSensorPin), isr_rotation, FALLING);
  // Serial.println(ESP.getResetReason());
  // Serial.println(serverErrorCount);

  float memoryDirOffset = EEPROM_readFloat(dirOffsetMemAddress);
  dirOffset = isnan(memoryDirOffset) ? 0 : memoryDirOffset;

  ledBlink(5, 500);
}

void loop()
{
    delay(1000);
    if(offsetMode) {
      readWindDir();
      Serial.println(windDirAngle);

      ledBlink();
    }else{
      unsigned long passedTimeSinceSend = millis() - lastSendData;
      if(passedTimeSinceSend >= sendDataInterval) {
        rotationTime = passedTimeSinceSend;
        readVoltage();
        readTempAndHumid();
        fillDataForServer();
        sendHttp();
        lastSendData = millis();
      }

      readWindDir();
      if(windDirBuffCount >= windDirBuffSize) {
        windDirBuffCount = 0;
      }
      windDirBuff[windDirBuffCount] = String(windDirAngle, 2);
      windDirBuffCount++;
    }
    
    handleButtonPress();
    
}

void handleButtonPress(){
  unsigned long start = millis();
  unsigned long pressTime = 0;

  while(digitalRead(Button) == 1){
    pressTime = millis() - start;
    if(pressTime >= 5000 && offsetMode == false){ 
      offsetMode = true;
      ledBlink(10, 50);
      delay(1000);
      break;
    }else if (pressTime > 500 && offsetMode == true){
      EEPROM_writeFloat(dirOffsetMemAddress, windDirAngleRaw);
      dirOffset = windDirAngleRaw;
      offsetMode = false;
      digitalWrite(LED_BUILTIN, HIGH);
      ledBlink(10, 100);
      break;
    }
    yield();
  }
  
  Serial.println(offsetMode);
  Serial.println(pressTime);
}


void ledBlink(){
  digitalWrite(LED_BUILTIN, ledTrigger);
  ledTrigger = !ledTrigger;
}

void ledBlink(int times, int ms){
  for(int i = 0; i < times; i++){
    digitalWrite(LED_BUILTIN, LOW);
    delay(ms);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(ms);
  }
}


void sendHttp(){
  Serial.println(F("AT"));
  delay(500);
  Serial.println(F("AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r"));
  delay(500);
  Serial.println(F("AT+SAPBR=3,1,\"APN\",\"internet.vivacom.bg\"\r"));
  delay(500);
  Serial.println(F("AT+SAPBR=1,1\r"));
  delay(500);
  Serial.println(F("AT+HTTPINIT\r"));
  delay(500);
  Serial.println(F("AT+HTTPPARA=\"URL\",\"http://46.55.200.199/Send\"\r"));
  delay(500);
  Serial.println(F("AT+HTTPPARA=\"CONTENT\",\"application/json\"\r"));
  delay(500);
  Serial.print(F("AT+HTTPDATA="));
  Serial.print(measureJson(dataToServer));
  Serial.println(",1500\r");
  delay(500);
  serializeJson(dataToServer, Serial); // Feed Data into AT+HTTPDATA
  Serial.println();
  delay(750);
  //serialFlush();
  delay(500);
  Serial.println(F("AT+HTTPACTION=1\r"));
  delay(2000);
  Serial.println(F("AT+HTTPREAD=0,300\r"));
  serialFlush();
  delay(500);
  String httpData = Serial.readString();
  //Serial.println(httpData);
  httpData = httpData.substring(httpData.indexOf('{'), httpData.lastIndexOf('}') + 1);
  //Serial.println(httpData);
  handleServerResponse(httpData);
  //Serial.println(F("AT+CSCLK=2\r"));
}

void handleServerResponse(String httpData) {
  dataFromServer.clear();
  DeserializationError error = deserializeJson(dataFromServer, httpData);
  if(error) {
    serverErrorCount++;
    //Serial.println(serverErrorCount);
    //Serial.println("Error");
    if(serverErrorCount == 10 ) { // 10 tries to connect to server
      saveInRTCMem(serverErrorCount, 1);
      enterSleep(10); // 10 min
    }else if(serverErrorCount >= 20) { //20 tries to connect to server
      saveInRTCMem(serverErrorCount, 1);
      enterSleep(20); // 20 min
    }
    else if(serverErrorCount >= 100) { //100 tries to connect to server
      saveInRTCMem(serverErrorCount, 1);
      enterSleep(360); // 360 = 6 HOUR
    }
  }else {
    serverErrorCount = 0;
    saveInRTCMem(serverErrorCount, 1);
    String mode = dataFromServer["mode"];
    short type = (short)dataFromServer["type"];
    //Serial.println(mode);
    if(mode == F("ps")){
      if(type == 0){
        enterSleep((short)dataFromServer["value"]);
      }
      
      short delay;
      switch(type){
        case 1: delay = 10;break; // 10min
        case 2: delay = 30;break; // 30min
        case 3: delay = 60;break; // 1hour
        case 4: delay = 120;break; // 2hour
        case 5: delay = 180;break; // 3hour
        case 6: delay = 360;break; // 6hour
        case 7: delay = 480;break; // 8hour
        case 8: delay = 600;break; // 10hour
      }
      enterSleep(delay);
    }
  }
}

void fillDataForServer() {
  dataToServer.clear();
  copyArray(windDirBuff, dataToServer["dd"]);
  dataToServer["rt"] = rotationTime; 
  dataToServer["r"] = rotations; 
  rotations = 0;
  dataToServer["h"] = String(dhtHum, 2); 
  dataToServer["t1"] = String(dhtTemp, 2); 
  dataToServer["v"] = String(voltage, 2);
  dataToServer["Id"] = "43123546";
}

void readVoltage(){
    float voltageOut = analogRead(voltagePin) * (3.3 / 1023.0);
    voltage = voltageOut * ((R1 + R2) / R1);
}

void readWindDir(){
  if((lastReadDirAngle - millis()) >= 1500){
    windDirAngleRaw = convertRawAngleToDegrees(ams5600.getRawAngle());\
    windDirAngle = fmod(windDirAngleRaw - dirOffset, 360.0);
    if (windDirAngle < 0) {
      windDirAngle += 360.0;
    }
    // Serial.print("angle - ");
    // Serial.println(windDirAngleRaw);
    // Serial.print("dirOffset - ");
    // Serial.println(dirOffset);
    Serial.print("windDirAngle - ");
    Serial.println(windDirAngle);
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
      serverErrorCount = 0;
      saveInRTCMem(serverErrorCount, 1);
      return;
    }
  }

  if(sleepMin >= 60 && !inSetup){
    hoursToSleep = (sleepMin / 60);
    saveInRTCMem(hoursToSleep, 5);
    sleepMin = 60;
  }

  digitalWrite(SensorPower, LOW);
  ESP.deepSleep((sleepMin * 6) *1e7, WAKE_RF_DISABLED);
}

void serialFlush(){
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
}

void EEPROM_writeFloat(int address, float value) {
  EEPROM.put(address, value);
  EEPROM.commit();
}

float EEPROM_readFloat(int address) {
  float value;
  EEPROM.get(address, value);
  return value;
}
