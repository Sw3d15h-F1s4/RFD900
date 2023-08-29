/**
D1
D2 - +3.3V for other shit
D3
D4 - +5V for MCU and LSM303
D5 - +5V for RADIO
D6 - LED1 - 45 (PD2) - GPS
D7 - LED2 - 46 (PD3) - SD CARD
D8 - same as D2
D9 - same as D4
D10 - LED3 - 47 (PD4) - LSM303 accel
D11 - LED4 - 48 (PD5) - LSM303 mag
D12 - LED5 - 49 (PD6) - GPS status
D13 - LED6 - 50 (PD7) - ticker
**/

#include "main.h"
//**************************** SETUP ***************************
void setup() {
  // Set LED pins as outputs
  pinMode(PIN_PD2, OUTPUT); // GPS Status.
  pinMode(PIN_PD3, OUTPUT); // SD Card Status.
  pinMode(PIN_PD4, OUTPUT); // LSM303 Accelerometer Status
  pinMode(PIN_PD5, OUTPUT); // LSM303 Magnetometer Status
  pinMode(PIN_PD6, OUTPUT); // GPS Fix Type
  pinMode(PIN_PD7, OUTPUT); // Tick

  // Set baud rates for serial connections
  Serial.begin(57600);  // Radio
  Serial2.begin(38400); // GPS

  // Set up I2C communication
  Wire.begin();

  // Device Setup
  gpsSetup();
  sdCardSetup(); // Also creates header for .csv
  accelMagSetup();
}
//**************************** Initilizations ******************
void gpsSetup() {
  pulseLED(PIN_PD2, 4, 100);
  do {
    pulseLED(PIN_PD2, 1, 500);
    Serial.println("GPS: trying 38400 baud");
    Serial2.begin(38400);
    if (myGPS.begin(Serial2) == true)
      break;
    delay(100);
    Serial.println("GPS: trying 9600 baud");
    Serial2.begin(9600);

    if (myGPS.begin(Serial2) == true) {
      Serial.println("GPS: connected at 9600 baud, switching to 38400");
      myGPS.setSerialRate(38400);
      delay(100);
    } else {
      // myGPS.factoryReset();
      delay(1500); // Wait a bit before trying again to limit the Serial output
    }
  } while (1);
  Serial.println("GPS serial connected");
  myGPS.setUART1Output(COM_TYPE_UBX); // Set the UART port to output UBX only
  myGPS.setI2COutput(COM_TYPE_UBX); // Set the I2C port to output UBX only (turn
                                    // off NMEA noise)
  myGPS.setDynamicModel(DYN_MODEL_AIRBORNE1g);   // Putting the GPS in flight mode.(airborne with
                               // <2g acceleration)
  myGPS.saveConfiguration();   // Save the current settings to flash and BBR
  digitalWrite(PIN_PD2, HIGH); // Turn the GPS Status LED on.
}

void sdCardSetup() {
  pulseLED(PIN_PD3, 4, 100);

  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present"); // don't do anything more:
    while (1) {
      pulseLED(PIN_PD3, 1, 500);
      if (SD.begin(chipSelect))
        break;
    }
  }

  Serial.println("card initialized.");
  // Create header for our CSV file
  File dataFile = SD.open("payload.csv", FILE_WRITE);
  dataFile.println("PacketNumber,SIV,FixType,Latitude,Longitude,Altitude,Year,"
                   "Month,Day,Hour,Min,Sec,NNV,NEV,NDV,Battery,3v3 Supply,5v "
                   "Supply,Radio Supply,Analog Internal,Analog "
                   "External,AltimeterTemp,Digital Internal,Digital "
                   "External,Pressure,Accel A,Accel Y,AccelZ,Pitch,Roll,Yaw");
  dataFile.close();
  digitalWrite(PIN_PD3, HIGH);
}


void accelMagSetup() {
  pulseLED(PIN_PD4, 4, 100);

  if (!accel.begin()) {
    Serial.println("LSM303 not detected");
    while (1)
      pulseLED(PIN_PD4, 1, 500);
  }
  digitalWrite(PIN_PD4, HIGH);

  pulseLED(PIN_PD5, 4, 100);
  if (!mag.begin()) {
    Serial.println("LSM303AGR not detected");
    while (1)
      pulseLED(PIN_PD5, 1, 500);
  }
  accel.setRange(LSM303_RANGE_2G); // set accel range (2, 4, 8, 16)
  accel.setMode(
      LSM303_MODE_NORMAL); // set accel mode (normal, low power, high res)
  digitalWrite(PIN_PD5, HIGH);
}


void pulseLED(int pin, int count, int time) {
  for (int i = 0; i < count; i++) {
    digitalWrite(pin, HIGH);
    delay(time);
    digitalWrite(pin, LOW);
    delay(time);
  }
}


void Read_GPS() {
  SIV = myGPS.getSIV();
  FixType = myGPS.getFixType();
  latitude = myGPS.getLatitude();
  longitude = myGPS.getLongitude();
  altitude = myGPS.getAltitude();
  year = myGPS.getYear();
  month = myGPS.getMonth();
  day = myGPS.getDay();
  hour = myGPS.getHour();
  minute = myGPS.getMinute();
  sec = myGPS.getSecond();
  NedNorthVel = myGPS.getNedNorthVel();
  NedEastVel = myGPS.getNedEastVel();
  NedDownVel = myGPS.getNedDownVel();
}


void LED_Fix_Type() {
  if (FixType == 0) {
    digitalWrite(PIN_PD6, LOW);
  } else if (FixType == 2) {
    digitalWrite(PIN_PD6, blink_GPS);
    blink_GPS = !blink_GPS;
  } else if (FixType == 3) {
    digitalWrite(PIN_PD6, HIGH);
  }
}
void Read_Battery_Voltage() {
  float ADC_Constant = 0.004882812; // This is 5/1024
  for (int i = 0; i < 100; i++) {
    Bat = Bat + analogRead(BATT_PIN);
  }
  Bat = Bat / 100;          // Averges the value
  Bat = Bat * ADC_Constant; // Converts to voltage for 5v Arduino
}
void Read_3v3_Supply() {
  int i;
  float ADC_Constant = 0.004882812; // This is 5/1024
  for (i = 0; i < 100; i++) {
    t3v3 = t3v3 + analogRead(T3v3_PIN);
  }
  t3v3 = t3v3 / 100;          // Averges the value
  t3v3 = t3v3 * ADC_Constant; // Converts to voltage for 5v Arduino
}
void Read_5v_Supply() {
  int i;
  float ADC_Constant = 0.004882812; // This is 5/1024
  for (i = 0; i < 100; i++) {
    f5v = f5v + analogRead(F5vS_PIN);
  }
  f5v = f5v / 100;          // Averges the value
  f5v = f5v * ADC_Constant; // Converts to voltage for 5v Arduino
}
void Read_5vI_Supply() {
  int i;
  float ADC_Constant = 0.004882812; // This is 5/1024
  for (i = 0; i < 100; i++) {
    f5vI = f5vI + analogRead(F5vI_PIN);
  }
  f5vI = f5vI / 100;          // Averges the value
  f5vI = f5vI * ADC_Constant; // Converts to voltage for 5v Arduino
}
void Read_Aint_Temp() {
  int i;
  float val = 0;
  float ADC_Constant = 4.88281; // This is 5000/1024
  for (i = 0; i < 100; i++) {
    val = val + analogRead(ITMP_PIN);
  }
  val = val / 100;                 // Averges the value
  val = val * ADC_Constant;        // Converts to voltage for 5v Arduino
  Aint = ((val - 1034) / (-5.58)); // Converts to Temperature in C for a
                                   // calibration range of 0 to 100 C
}
void Read_Aext_Temp() {
  int i;
  float val = 0;
  float ADC_Constant = 4.88281; // This is 5000/1024
  for (i = 0; i < 100; i++) {
    val = val + analogRead(ETMP_PIN);
  }
  val = val / 100;                 // Averges the value
  val = val * ADC_Constant;        // Converts to voltage for 5v Arduino
  Aext = ((val - 1034) / (-5.58)); // Converts to Temperature in C for a
                                   // calibration range of 0 to 100 C
}
void Read_Pressure_Module() {
  sensor.ReadProm();
  sensor.Readout();
  Pres_Temp = (sensor.GetTemp() / 100);
  pressure = (sensor.GetPres() / 100);
}
void Read_Dint_Temp() {
  uint8_t data[2];
  int16_t datac;
  Wire.beginTransmission(0x48);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(10);
  Wire.requestFrom(0x48, 2);
  if (Wire.available() <= 2) { // checks bytes
    data[0] = Wire.read();
    data[1] = Wire.read();
    datac = ((data[0] << 8) | data[1]); // combines data to 16 bit
    Dint = datac * 0.0078125;           // converts to celsius (7.8125 mC res);
  }
}
void Read_Dext_Temp() {
  uint8_t data[2];
  int16_t datac;
  Wire.beginTransmission(0x49);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(10);
  Wire.requestFrom(0x49, 2);
  if (Wire.available() <= 2) { // checks bytes
    data[0] = Wire.read();
    data[1] = Wire.read();
    datac = ((data[0] << 8) | data[1]); // combines data to 16 bit
    Dext = datac * 0.0078125;           // converts to celsius (7.8125 mC res);
  }
}
void Read_Accelerometer_Mag() {
  sensors_event_t event;
  accel.getEvent(&event); // gets measurement
  Acceleration_X = event.acceleration.x;
  Acceleration_Y = event.acceleration.y;
  Acceleration_Z = event.acceleration.z;

  double a_x = event.acceleration.x; // gets accel
  double a_y = event.acceleration.y;
  double a_z = event.acceleration.z;
  double xg = (a_x) / 9.80665; // calculates accel to g's
  double yg = (a_y) / 9.80665;
  double zg = (a_z) / 9.80665;
  double pitch =
      (180 / M_PI) *
      atan2(xg, sqrt(sq(yg) + sq(zg))); // calculate pitch from accel values
  double roll =
      (180 / M_PI) *
      atan2(yg, sqrt(sq(xg) + sq(zg))); // calculate roll from the accel values
  Pitch = (180 / M_PI) * atan2(xg, sqrt(sq(yg) + sq(zg)));
  Roll = (180 / M_PI) * atan2(yg, sqrt(sq(xg) + sq(zg)));

  mag.getEvent(&event);                   // gets measurement
  double magx = event.magnetic.x - 21.67; // Inside = 16.28
  double magy = event.magnetic.y + 18.45; // Inside = -17.40
  double magz = event.magnetic.z - 6.9;   // Inside = 6.83
  double pitchr = (pitch * M_PI) / 180;
  double rollr = (roll * M_PI) / 180;
  double a = magx * cos(pitchr) + sin(pitchr) * sin(rollr) * magy +
             magz * cos(rollr) * sin(pitchr);
  double b = magy * cos(rollr) - magz * sin(rollr);
  double yaw = 180 * atan2(-b, a) / M_PI;
  Yaw = 180 * atan2(-b, a) / M_PI;
  if (yaw < 0) {
    yaw = 360 + yaw;
  }
}
//***************Data Sending & Storing ************************
void Store_Data() {
  File dataFile = SD.open("payload.csv", FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) { // Write our data to the csv file
    dataFile.print(String(packet));
    dataFile.print(",");
    dataFile.print(String(SIV));
    dataFile.print(",");
    dataFile.print(String(FixType));
    dataFile.print(",");
    dataFile.print(String(latitude));
    dataFile.print(",");
    dataFile.print(String(longitude));
    dataFile.print(",");
    dataFile.print(String(altitude));
    dataFile.print(",");
    dataFile.print(String(year));
    dataFile.print(",");
    dataFile.print(String(month));
    dataFile.print(",");
    dataFile.print(String(day));
    dataFile.print(",");
    dataFile.print(String(hour));
    dataFile.print(",");
    dataFile.print(String(minute));
    dataFile.print(",");
    dataFile.print(String(sec));
    dataFile.print(",");
    dataFile.print(String(NedNorthVel));
    dataFile.print(",");
    dataFile.print(String(NedEastVel));
    dataFile.print(",");
    dataFile.print(String(NedDownVel));
    dataFile.print(",");
    dataFile.print(String(Bat));
    dataFile.print(",");
    dataFile.print(String(t3v3));
    dataFile.print(",");
    dataFile.print(String(f5v));
    dataFile.print(",");
    dataFile.print(String(f5vI));
    dataFile.print(",");
    dataFile.print(String(Aint));
    dataFile.print(",");
    dataFile.print(String(Aext));
    dataFile.print(",");
    dataFile.print(String(Pres_Temp));
    dataFile.print(",");
    dataFile.print(String(Dint));
    dataFile.print(",");
    dataFile.print(String(Dext));
    dataFile.print(",");
    dataFile.print(String(pressure));
    dataFile.print(",");
    dataFile.print(String(Acceleration_X));
    dataFile.print(",");
    dataFile.print(String(Acceleration_Y));
    dataFile.print(",");
    dataFile.print(String(Acceleration_Z));
    dataFile.print(",");
    dataFile.print(String(Pitch));
    dataFile.print(",");
    dataFile.print(String(Roll));
    dataFile.print(",");
    dataFile.println(String(Yaw));

    dataFile.close();
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error");
  }
  dataFile.close();
}
void Send_Data() {
  Serial.print(packet);
  Serial.print(',');
  Serial.print(SIV);
  Serial.print(',');
  Serial.print(FixType);
  Serial.print(',');
  Serial.print(latitude);
  Serial.print(',');
  Serial.print(longitude);
  Serial.print(',');
  Serial.print(altitude);
  Serial.print(',');
  Serial.print(year);
  Serial.print(',');
  Serial.print(month);
  Serial.print(',');
  Serial.print(day);
  Serial.print(',');
  Serial.print(hour);
  Serial.print(',');
  Serial.print(minute);
  Serial.print(',');
  Serial.print(sec);
  Serial.print(',');
  Serial.print(NedNorthVel);
  Serial.print(',');
  Serial.print(NedEastVel);
  Serial.print(',');
  Serial.print(NedDownVel);
  Serial.print(',');
  Serial.print(Bat);
  Serial.print(',');
  Serial.print(t3v3);
  Serial.print(',');
  Serial.print(f5v);
  Serial.print(',');
  Serial.print(f5vI);
  Serial.print(',');
  Serial.print(Aint);
  Serial.print(',');
  Serial.print(Aext);
  Serial.print(',');
  Serial.print(Pres_Temp);
  Serial.print(',');
  Serial.print(Dint);
  Serial.print(',');
  Serial.print(Dext);
  Serial.print(',');
  Serial.print(pressure);
  Serial.print(',');
  Serial.print(Acceleration_X);
  Serial.print(',');
  Serial.print(Acceleration_Y);
  Serial.print(',');
  Serial.print(Acceleration_Z);
  Serial.print(',');
  Serial.print(Pitch);
  Serial.print(',');
  Serial.print(Roll);
  Serial.print(',');
  Serial.print(Yaw);
  Serial.print(',');

  Serial.println("END");
}

//*********************** MAIN LOOP ****************************
void loop() {
  pulseLED(PIN_PD7, 1, 100);
  Read_GPS();
  LED_Fix_Type();
  Read_Battery_Voltage();
  Read_3v3_Supply();
  Read_5v_Supply();
  Read_5vI_Supply();
  Read_Aint_Temp();
  Read_Aext_Temp();
  Read_Pressure_Module();
  Read_Dint_Temp();
  Read_Dext_Temp();
  Read_Accelerometer_Mag();
  Store_Data();
  Send_Data();
  // Send_Data();
  // Keep track of packets being sent
  packet = packet + 1;
}
