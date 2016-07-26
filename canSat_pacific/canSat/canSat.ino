/* CanSat 2016
 *  
 *  This sketch controls the Seeeduino plus Pacific custom board
 *  for the Sierra High and Pacific CanSat project of 2016.
 *  
 *  The specific hardware controlled:
 *    IMU: BMX055
 *      Accelerometer I2C = 0x18
 *      Gyro I2C = 0x69
 *      Magnatometer I2C = 0x10
 *    Pressure: BMP280
 *      I2C = 0x76
 *    Humidity: HDC1080
 *      I2C = 0x40
 *    Ozone: MICS-2614
 *      ADC on PC1 / A1
 *    Methane:
 *      ADC on PC0 / A0
 *    Geiger:
 *      GPIO on PD2 / 2
 *    Real Time Clock:
 *      I2C = 0x68
 *    SD Card:
 *      SPI
 *      Chip Select = PB2 / 10
 */

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <avr/wdt.h>

#include "RTClib.h"
#include "HDC1050.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_BMP280.h"
#include "bmx055.h"

// Define addresses and lines
#define SDCARD_CHIP_SELECT      10     // chip select pin for SD Card on Seeeduino
#define OUR_BMP280_ADDRESS      0x76   // Address of pressure sensor (AdaFruit has own def
  
// Global Variables
RTC_DS3231 rtc;
bool rtcFlag = false;

DateTime now;
String fileName = "log";

HDC1050 hdcSensor;
Adafruit_BMP280 bme;
bool pressureFlag = false;
bool imuFlag = false;

unsigned short geigerClicks = 0;

void setup() {
  //Begin Watchdog Setup
  cli();
  wdt_reset();
  WDTCSR = (1<<WDCE)|(1<<WDE);
  WDTCSR = (1<<WDIE)|(1<<WDE)|(1<<WDP3);
  sei();
  //End Watchdog Setup

  // Open serial communication for debug purposes
  Serial.begin(9600);
  
  // Open I2C
  Wire.begin();

  // Setup SD Card
  Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  // EAB - switch to while as if there is no card, nothing matters
  if (!SD.begin(SDCARD_CHIP_SELECT)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }

  // Open syslog and write
  File sysFile = SD.open("syslog.txt", FILE_WRITE);
  if(sysFile) {
        sysFile.println("-------------------------------------");
  }

  // Setup RTC
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    rtcFlag = false;
  } else {
    rtcFlag = true;
  
    if (rtc.lostPower()) {
      Serial.println(F("RTC lost power, setting the date and time"));
      sysFile.println(F("RTC lost power, setting the date and time!"));
      // following line arbitrarily sets the RTC to the middle of August
      rtc.adjust(DateTime(2016,8,15, 0,0,0));
    }
  }

  // Setup Humidity sensor
  //hdcSensor.turnOnHeater(true);                 // Heats in measurement mode for testing or condensation
  hdcSensor.setTemperatureRes(HDC1050::T_RES_14); // 14 bit resolution on temp measurement
  hdcSensor.setHumidityRes(HDC1050::H_RES_14);    // 14 bit resolution on humidity measurement
  hdcSensor.updateConfigRegister();               // Actually set the resulotions
  sysFile.println(F("Initialized humidity sensor"));

  // Setup Pressure sensor
  if (bme.begin(OUR_BMP280_ADDRESS)) {  
    sysFile.println(F("Initialized pressure sensor"));
    pressureFlag=true;
  } else {
    sysFile.println(F("Could not initialize pressure sensor"));
    pressureFlag=false;
  }

  // Setup IMU sensor
  sysFile.print(F("BMX055 accelerometer..."));
  byte c = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_WHOAMI);  // Read ACC WHO_AM_I register for BMX055
  sysFile.println(c, HEX);
  sysFile.print(F("BMX055 gyroscope..."));
  byte d = readByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_WHOAMI);  // Read GYRO WHO_AM_I register for BMX055
  sysFile.println(d, HEX);
  sysFile.print(F("BMX055 magnetometer..."));
  writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_PWR_CNTL1, 0x01); // wake up magnetometer first thing
  delay(100);
  byte e = readByte(BMX055_MAG_ADDRESS, BMX055_MAG_WHOAMI);  // Read MAG WHO_AM_I register for BMX055
  sysFile.println(e, HEX);
  if ((c == 0xFA) && (d == 0x0F) && (e == 0x32)) // WHO_AM_I should always be ACC = 0xFA, GYRO = 0x0F, MAG = 0x32
  {  
     imuFlag = true;
     sysFile.println(F("BMX055 is online..."));

     initBMX055(); 
     sysFile.println(F("BMX055 initialized for active data mode....")); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
  } else {
    imuFlag = false;
  }

  //Setup Geiger
  pinMode(2,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2),geiger_click,RISING);
  
  // Done with setup
  sysFile.close();

  // Create filename for logging data
  if(rtcFlag){
    DateTime now = rtc.now();
    fileName += String(now.month());
    fileName += String(now.day());
    fileName += ".txt";
  } else {
    fileName +="no.txt";
  }

  // open the file to start
  File dataFile = SD.open(fileName, FILE_WRITE);
  if(dataFile){
     dataFile.println("-------------------------------------");
     dataFile.print(F("YYYY, MM, DD, HH, MM, SS, 1050_T(C), 1050_Humidity, 280_T(C), Pressure(Pa)"));
     dataFile.print(F(", Accel_x(g), Accel_y, Accel_z, Gyro_x(degrees/sec), Gyro_y, Gyro_z, Mag_x(milliGauss), "));
     dataFile.println(F("Mag_y, Mag_z, Mag_hall (raw), Methane(V), Ozone(Ohms), Geiger (Clicks)"));
     dataFile.close();
  }

}

void loop() {
  // open the file for logging current data
  File dataFile = SD.open(fileName, FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    // Print day and time to file
    if(rtcFlag){
      DateTime now = rtc.now();
      dataFile.print(now.year());
      dataFile.print(", ");
      dataFile.print(now.month());
      dataFile.print(", ");
      dataFile.print(now.day());
      dataFile.print(", ");
      dataFile.print(now.hour());
      dataFile.print(", ");
      dataFile.print(now.minute());
      dataFile.print(", ");
      dataFile.print(now.second());
      dataFile.print(", ");
    } else {
      dataFile.print("rtc error,");
    }

    // Measure Humidity
    float tc, h;
    hdcSensor.getTemperatureHumidity(tc, h);
    dataFile.print(tc);
    dataFile.print(", ");
    dataFile.print(h);
    dataFile.print(", ");

    // Measure Pressure
    if(pressureFlag){
      dataFile.print(bme.readTemperature());
      dataFile.print(", ");
      dataFile.print(bme.readPressure());
      dataFile.print(", ");
    } else {
      dataFile.print("pressure error,");
    }

    // Measure IMU
    if(imuFlag) {
      int16_t imuData[4];  // Stores the 16-bit signed imu sensor output

      readAccelData(imuData);  // Read the x/y/z adc values
      dataFile.print((float)imuData[0]*(2.0/2048.0));
      dataFile.print(", ");
      dataFile.print((float)imuData[1]*(2.0/2048.0));
      dataFile.print(", ");
      dataFile.print((float)imuData[2]*(2.0/2048.0));
      dataFile.print(", ");
      readGyroData(imuData);  // Read the x/y/z adc values
      dataFile.print((float)imuData[0]*(124.87/32768.0));
      dataFile.print(", ");
      dataFile.print((float)imuData[1]*(124.87/32768.0));
      dataFile.print(", ");
      dataFile.print((float)imuData[2]*(124.87/32768.0));
      dataFile.print(", ");
      readMagData(imuData);  // Read the x/y/z adc values
      dataFile.print((float)imuData[0]*(1./1.6));
      dataFile.print(", ");
      dataFile.print((float)imuData[1]*(1./1.6));
      dataFile.print(", ");
      dataFile.print((float)imuData[2]*(1./1.6));
      dataFile.print(", ");
      dataFile.print(imuData[3]);
      dataFile.print(", ");
    } else {
      dataFile.print("imu error, ");
    }

    // Methane
    int temp = analogRead(A0);
    float methaneRes = (3.3*temp)/1024.0;
    //methaneRes = (4700/(3.3 - methaneRes))*methaneRes;
    dataFile.print(methaneRes);
    dataFile.print(", ");

    // Ozone
    temp = analogRead(A1);
    float ozoneRes = (3.3*temp)/1024.0;
    ozoneRes = (10000/(3.3 - ozoneRes))*ozoneRes;
    dataFile.print(ozoneRes);
    dataFile.print(", ");

    //Geiger
    dataFile.println(geigerClicks);
    geigerClicks = 0;
    
    dataFile.close();
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println(F("Error opening datalog.txt"));
  }

  delay(500);

  wdt_reset(); //Call this to reset the watchdog timer
}

void geiger_click(){
  geigerClicks++;
}
