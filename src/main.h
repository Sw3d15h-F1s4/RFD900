
#include <Arduino.h>

#include <math.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>  //http://librarymanager/All#SparkFun_u-blox_GNSS
#include <MS5xxx.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_LIS2MDL.h> //#include <Adafruit_LSM303AGR_Mag.h>
#include <Adafruit_Sensor.h>

SFE_UBLOX_GNSS myGPS;
MS5xxx sensor(&Wire);

//Pin Assignments
#define BATT_PIN PIN_PF1 //Battery Monitor Pin 
#define T3v3_PIN PIN_PK1 //3v3 rail monitor
#define F5VS_PIN PIN_PK2 //5v rail monitor
#define F5vI_PIN PIN_PK3 //5v internal rail monitor
#define ITMP_PIN PIN_PF2 //Internal Temperature monitor
#define ETMP_PIN PIN_PF0 //External Temperature monitor
#define SDCS_PIN PIN_PB0 //SD card Chip Select pin

//Packet information
int packet = 0;
//LED Toggle
bool blink_GPS = true;

//GPS Data
byte SIV;
byte FixType;
long latitude;
long longitude;
long altitude;
int year;
int month;
int day;
int hour;
int minute;
int sec;
long NedNorthVel;
long NedEastVel;
long NedDownVel;

//Power Data
float Bat = 0;
float t3v3 = 0;
float f5v = 0;
float f5vI = 0;

//Temperature Data
float Aint;
float Aext;
float Pres_Temp;
float Dint;
float Dext;

//Pressure Data
float pressure;

//Digital Temperature Sensor Constants and Variables
const int tmp_addr = 0x48;
const int temp_addr = 0x00;
const int config_addr = 0x01;
const int hlim_addr = 0x02;
const int llim_addr = 0x03;
const int eeprom_unlock_addr = 0x04;
const int deviceID_addr = 0x0F;
const uint8_t highlimH = B00001101;  // High byte of high lim
const uint8_t highlimL = B10000000;  // Low byte of high lim - High 27 C
const uint8_t lowlimH = B00001100;   // High byte of low lim
const uint8_t lowlimL = B00000000;   // Low byte of low lim - Low 24 C

//Accelerometer and Mag Constants and Variables
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(0x19);
Adafruit_LIS2MDL mag = Adafruit_LIS2MDL(0x1E);
//Adafruit_LSM303AGR_Mag_Unified mag = Adafruit_LSM303AGR_Mag_Unified(0x1E);

//Accelerometer Data
double Acceleration_X;
double Acceleration_Y;
double Acceleration_Z;
double Pitch;
double Roll;
double Yaw;


void gpsSetup();

void sdCardSetup();

void accelMagSetup();

void pulseLED(int pin, int count, int time);

void Read_GPS();

void LED_Fix_Type();

void Read_Battery_Voltage();

void Read_3v3_Supply();

void Read_5vI_Supply();

void Read_Aint_Temp();

void Read_Aext_Temp();

void Read_Pressure_Module();

void Read_Dint_Temp();

void Read_Dext_Temp();

void Read_Accelerometer_Mag();

void Store_Data();

void Send_Data();

void loop();
void setup();
