#include <LiquidCrystal.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <avr/eeprom.h>
// Chars for lcd
byte char_clock[8] = {
 B01110,
 B10011,
 B11011,
 B10101,
 B10001,
 B10001,
 B01110,
};
byte char_cubeTepm[8] = {
 B00100,
 B01010,
 B01010,
 B01110,
 B10001,
 B10001,
 B01110,
};
//Ardubrew config For LCD 8,9,4,5,6,7 - reserved 
const int oneWire_pin = 2;
const int heater_pin = A2; // before were 3;
const int valve_pin = 11;
const int water_pin = 13;
const int relay3_pin = 12;
const int relay4_pin = 10;
const int flowMeter_pin = 3;
const int waterAlert_pin = A1;
// Memory map
const byte cubeTreshEeprom = 0; // 0-1 byte
const byte cubeThermometerAddrEeprom = 1; //1-8 byte
const byte valveThermometerAddrEeprom = 9; //9-16 byte
const byte VaporAlertThermometerAddrEeprom = 17; //17-24 byte
const byte valveDeltaEeprom = 25; //25 byte
const byte waterTreshEeprom = 26; // 26 byte
const byte VaporAlertTreshEeprom = 27; // 27 byte

bool statusStabilized = 0;
unsigned long stabilizeTime = 300000;
bool valveOpen_state=0;
bool waterOpen_state=0;
float cubeTemp = -100;
byte cubeTresh = 99;
byte waterTresh = 60  ;
float valveTemp = -100;
float valveDelta = 2;
byte heaterState = 0;
bool valveState = 0;
bool waterState = 0;
float tempNoise = 0.13;
unsigned long valveDelay = 300000;


//LCD config
int  adc_key_val[5] ={30, 150, 360, 535, 760 };
int NUM_KEYS = 5;
int adc_key_in;
int key=-1;
int oldkey=-1;
const byte upButton = 1;
const byte downButton = 2;
const byte leftButton = 3;
const byte rightButton = 0;
const byte selectButton = 4;
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
//Dallas config
#define TEMPERATURE_PRECISION 12
OneWire oneWire(oneWire_pin);
DallasTemperature sensors(&oneWire);
DeviceAddress cubeThermometerAddr, valveThermometerAddr;
//Menu
int currButton =-1;
byte currMenu = 0;
int menuMin = 0;
int menuMax = 9;
String menuText[10] = {"1.Start Distil", "2.Stop dist temp", "3.Last Stats", "4.Valve Check", "5.Start Rectify", "6.Set cubeT addr", "7.Set vlveT addr", "8.Valve delta", "9.Stabilize", "10.Water Check"};
//Stats
int distTime = 0;
int distStopTemp = 0;


void setup() {
Serial.begin(9600);
lcd.begin(16, 2);
lcd.setCursor(0, 0);
lcd.print("Ardubrew by KVT");
delay(1000);
lcd.clear();
lcd.createChar(0, char_clock);
lcd.createChar(1, char_cubeTepm);
// init GPIO
pinMode(heater_pin, OUTPUT);
pinMode(valve_pin, OUTPUT);
pinMode(water_pin, OUTPUT);
pinMode(relay3_pin, OUTPUT);
pinMode(relay4_pin, OUTPUT);
digitalWrite(heater_pin, LOW);
digitalWrite(valve_pin, HIGH);
digitalWrite(water_pin, HIGH);
digitalWrite(relay3_pin, HIGH);
digitalWrite(relay4_pin, HIGH);


//init ds18b20
sensors.begin();
Serial.print("Locating devices...");
Serial.print("Found ");
Serial.print(sensors.getDeviceCount(), DEC);
Serial.println(" devices.");
Serial.print("Parasite power is: ");
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");
//if (!sensors.getAddress(cubeThermometerAddr, 0)) Serial.println("Unable to find address for Device 0");
//if (!sensors.getAddress(valveThermometerAddr, 1)) Serial.println("Unable to find address for Device 1");
Serial.print("Device 0 Address: ");
  printAddress(cubeThermometerAddr);
  Serial.println();
  Serial.print("Device 1 Address: ");
  printAddress(valveThermometerAddr);
  Serial.println();
sensors.setResolution(cubeThermometerAddr, TEMPERATURE_PRECISION);
sensors.setResolution(valveThermometerAddr, TEMPERATURE_PRECISION);
Serial.print("Device 0 Resolution: ");
Serial.print(sensors.getResolution(cubeThermometerAddr), DEC);
Serial.println();
Serial.print("Device 1 Resolution: ");
Serial.print(sensors.getResolution(valveThermometerAddr), DEC);
Serial.println();

//EEPROM LOAD
cubeTresh = eeprom_read_byte(cubeTreshEeprom);
for (uint8_t i = 0; i < 8; i++) { cubeThermometerAddr[i] = eeprom_read_byte(i+cubeThermometerAddrEeprom); };
for (uint8_t i = 0; i < 8; i++) { valveThermometerAddr[i] = eeprom_read_byte(i+valveThermometerAddrEeprom); };
}

void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

// function to print the temperature for a device
void printTemperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  if(tempC == DEVICE_DISCONNECTED_C) 
  {
    Serial.println("Error: Could not read temperature data");
    return;
  }
  Serial.print("Temp C: ");
  Serial.print(tempC);
}
float getTemperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  if(tempC == DEVICE_DISCONNECTED_C) 
  {
    Serial.println("Error: Could not read temperature data");
    return;
  }
  Serial.print("Temp C: ");
  Serial.print(tempC);
  return tempC;
}
// function to print a device's resolution
void printResolution(DeviceAddress deviceAddress)
{
  Serial.print("Resolution: ");
  Serial.print(sensors.getResolution(deviceAddress));
  Serial.println();
}

// main function to print information about a device
void printData(DeviceAddress deviceAddress)
{
  Serial.print("Device Address: ");
  printAddress(deviceAddress);
  Serial.print(" ");
  printTemperature(deviceAddress);
  Serial.println();
}

// Convert ADC value to key number
int get_key(unsigned int input)
{
  int k;
    
  for (k = 0; k < NUM_KEYS; k++)
  {
    if (input < adc_key_val[k])
    {
           
    return k;
        }
  }
    
    if (k >= NUM_KEYS)
        k = -1;     // No valid key pressed
    
    return k;
}


int getPress()
{
  adc_key_in = analogRead(0);    // read the value from the sensor  
  key = get_key(adc_key_in);            // convert into key press
  delay(300);    // wait for debounce time 
  oldkey = key;
  return key;     
 
}


void startDistill()
{
  unsigned long startMillis = millis();
  lcd.clear();
  lcd.setCursor(0, 0);lcd.print("Distilling ");
  lcd.setCursor(0, 1);
  digitalWrite(heater_pin, HIGH);
  
  while (cubeTemp<cubeTresh) {
    sensors.requestTemperatures();
    cubeTemp = getTemperature(cubeThermometerAddr);
    safetyCheck();
    lcd.setCursor(11, 0);lcd.print((millis()-startMillis)/60000);
    lcd.setCursor(0, 1);lcd.write(char(1)); lcd.print(cubeTemp);
    lcd.setCursor(10, 1);lcd.print(cubeTresh);

    
    if (cubeTemp>waterTresh) startWater();
    delay(1000);
  }
  digitalWrite(heater_pin, LOW);
  stopWater();
  distTime = trunc((millis()-startMillis)/60000);
  distStopTemp = cubeTemp;
  
  currMenu = 2; //Показать статистику
  
  
}

void startRectify()
{

  stabilize();
  
  unsigned long startMillis = millis();
  unsigned long valveDelayStartMillis = millis();
  sensors.requestTemperatures();
  cubeTemp = getTemperature(cubeThermometerAddr);
  valveTemp = getTemperature(valveThermometerAddr);
  float valveStopTemp = valveTemp+valveDelta;
  lcd.clear();
  lcd.setCursor(0, 0);lcd.print("Rectifying ");
  lcd.setCursor(0, 1);
  digitalWrite(heater_pin, HIGH);
  while (cubeTemp<cubeTresh) {
    sensors.requestTemperatures();
    cubeTemp = getTemperature(cubeThermometerAddr);
    valveTemp = getTemperature(valveThermometerAddr);
    safetyCheck();
    if ((valveTemp<valveStopTemp)&&((millis()-valveDelayStartMillis)>valveDelay)) valveState=valveOpen_state; 
    if (valveTemp>=valveStopTemp) {valveState=!valveOpen_state; valveDelayStartMillis = millis();};//delay(valveDelay);};
    processValve();
    
    lcd.setCursor(11, 0);lcd.print((millis()-startMillis)/60000);
    lcd.setCursor(0, 1);lcd.write(char(1)); lcd.print(cubeTemp);
    lcd.setCursor(8, 1); if (valveState==valveOpen_state) lcd.print("*"); else lcd.print("_");
    lcd.setCursor(10, 1);lcd.print(valveTemp);
    
    delay(1000);
  }
  digitalWrite(heater_pin, LOW);
  stopWater();
  distTime = trunc((millis()-startMillis)/60000);
  distStopTemp = cubeTemp;
  currMenu = 2; //Показать статистику
  
}

void moveMenu()
{
  if ((currButton == upButton) and (currMenu>menuMin)) {currMenu--;currButton =-1;return;};  
  if ((currButton == downButton) and (currMenu<menuMax)) {currMenu++;currButton =-1;return;};
  
  if ((currButton == selectButton) and (currMenu==0)) {startDistill();return;};  
  
  if ((currButton == rightButton) and (currMenu==1)) cubeTresh=cubeTresh+1;
  if ((currButton == leftButton) and (currMenu==1)) cubeTresh=cubeTresh-1;  
  if ((currButton == selectButton) and (currMenu==1)) eeprom_write_byte(cubeTreshEeprom, cubeTresh);  
  
  if ((currButton == rightButton) and (currMenu==3)) {digitalWrite(valve_pin, LOW);valveState=!valveOpen_state;}
  if ((currButton == leftButton) and (currMenu==3)) {digitalWrite(valve_pin, HIGH);valveState=valveOpen_state;}

  if ((currButton == selectButton) and (currMenu==4)) {startRectify();return;};

  if ((currButton == selectButton) and (currMenu==5)) {oneWire.reset_search(); if (!sensors.getAddress(cubeThermometerAddr, 0)) Serial.println("Unable to find address for Device 0"); for (uint8_t i = 0; i < 8; i++) { eeprom_write_byte(i+cubeThermometerAddrEeprom, cubeThermometerAddr[i]); }};

  if ((currButton == selectButton) and (currMenu==6)) {oneWire.reset_search(); if (!sensors.getAddress(valveThermometerAddr, 0)) Serial.println("Unable to find address for Device 0"); for (uint8_t i = 0; i < 8; i++) { eeprom_write_byte(i+valveThermometerAddrEeprom, valveThermometerAddr[i]); }};

  if ((currButton == rightButton) and (currMenu==7)) valveDelta=valveDelta+0.1;
  if ((currButton == leftButton) and (currMenu==7)) valveDelta=valveDelta-0.1;  
  //if ((currButton == selectButton) and (currMenu==7) eeprom_write_byte(16, valveDelta);  

   if ((currButton == selectButton) and (currMenu==8)) {stabilize(); return;};  

  if ((currButton == rightButton) and (currMenu==9)) {digitalWrite(water_pin, LOW);waterState=!waterOpen_state;}
  if ((currButton == leftButton) and (currMenu==9)) {digitalWrite(water_pin, HIGH);waterState=waterOpen_state;}

}
void processValve()
{
   if (valveState!=digitalRead(valve_pin)) digitalWrite(valve_pin, valveState);
  
}

void safetyCheck()
{
  
  
}

void startWater()
{
   digitalWrite(water_pin, waterOpen_state);
  
}

void stopWater()
{
   delay(60000);
   digitalWrite(water_pin, !waterOpen_state);
  
}

void emergencyStopWater()
{
   digitalWrite(water_pin, !waterOpen_state);
  
}

void stabilize()
{
  unsigned long startMillis = millis();
  unsigned long startStabilizeMillis = millis();
  
  float lastValveTemp;
  lcd.clear();
  lcd.setCursor(0, 0);lcd.print("Stabilizing ");
  lcd.setCursor(0, 1);
  digitalWrite(heater_pin, HIGH);
  statusStabilized=0;

  while (!statusStabilized) {
    sensors.requestTemperatures();
    valveTemp = getTemperature(valveThermometerAddr);
    cubeTemp = getTemperature(cubeThermometerAddr);
    safetyCheck();
    if (cubeTemp>waterTresh) startWater();
    if ((abs(valveTemp-lastValveTemp)<tempNoise)&&((millis()-startStabilizeMillis)>stabilizeTime)) {statusStabilized=1;  };
    if ((abs(valveTemp-lastValveTemp)>=tempNoise)&&(!statusStabilized)) {lastValveTemp=valveTemp; startStabilizeMillis=millis();};
    
    lcd.setCursor(11, 0);lcd.print((millis()-startMillis)/60000);
    lcd.setCursor(0, 1);lcd.write(char(1)); lcd.print(cubeTemp);
    lcd.setCursor(10, 1);lcd.print(valveTemp);
    
    delay(1000);
  }
  statusStabilized = 0;
  distTime = trunc((millis()-startMillis)/60000); 
  distStopTemp = cubeTemp; 
  currMenu = 2;
  
}

void printMenu()
{
  lcd.clear();
//  lcd.setCursor(14, 1);lcd.print(currButton);
  lcd.setCursor(0, 0); 
  lcd.print(menuText[currMenu]);
   if (currMenu==1) {
    lcd.setCursor(0, 1);
    lcd.print(cubeTresh);   
   }
   if (currMenu==2) {
    lcd.setCursor(0, 1); lcd.print("Time:"); lcd.print(distTime);  
    lcd.setCursor(8, 1); lcd.print("Tstop:"); lcd.print(distStopTemp);  
   }   
   if (currMenu==3) {
    sensors.requestTemperatures();
    valveTemp = getTemperature(valveThermometerAddr);
    lcd.setCursor(0, 1); lcd.print("V:"); lcd.print(valveState);  
    lcd.setCursor(5, 1); lcd.print("Tv:"); lcd.print(valveTemp); 
    //delay(1000); 
   }   
   if (currMenu==5) {
    lcd.setCursor(0, 1);  for (uint8_t i = 0; i < 8; i++) {if (cubeThermometerAddr[i] < 16) lcd.print("0"); lcd.print(cubeThermometerAddr[i], HEX);}  
      
   }   
   if (currMenu==6) {
    lcd.setCursor(0, 1);  for (uint8_t i = 0; i < 8; i++) {if (valveThermometerAddr[i] < 16) lcd.print("0"); lcd.print(valveThermometerAddr[i], HEX);}    
      
   }   
   if (currMenu==7) {
    lcd.setCursor(0, 1);
    lcd.print(valveDelta); 
   }

   if (currMenu==9) {
    sensors.requestTemperatures();
    cubeTemp = getTemperature(cubeThermometerAddr);
    lcd.setCursor(0, 1); lcd.print("W:"); lcd.print(waterState);  
    lcd.setCursor(5, 1); lcd.print("Tc:"); lcd.print(cubeTemp); 
    //delay(1000); 
   }   
}


void loop() {
//lcd.write(char(0));

currButton = getPress();
moveMenu();
printMenu();


delay(100);
}
