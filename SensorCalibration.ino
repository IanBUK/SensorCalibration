//#define TEST_MODE

#include "Adafruit_Sensor_Calibration.h"

// select either EEPROM or SPI FLASH storage:
//#ifdef ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM
  Adafruit_Sensor_Calibration_EEPROM cal;
//#else
//  Adafruit_Sensor_Calibration_SDFat cal;
//#endif

float hardIronTestValues[3];
float softIronTestValues[9];
float magneticField = 45.0;
float gyroZeroRotate[3];

// MotionCal uses an 80-byte packet format
const int calibrationDataSize = 80;
byte caldata[calibrationDataSize];
byte calcount = 0;

// Internal struct to unpack the MotionCal binary stream
struct __attribute__((packed)) MotionCalPacket {
  uint16_t header;     // 'uT' (117, 84)
  float offsets[9];    // Accel(3), Gyro(3), Mag(3)
  float softiron[9];   // 3x3 Matrix
  float field;         // Magnetic Field Strength
  uint16_t crc;        // CRC16
};

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  delay(100);

  if (!cal.begin()) {
    Serial.println("Failed to initialize calibration EEPROM");
    while (1) yield();
  }

#ifdef TEST_MODE
  performEEPROMTest();
#else
  if (cal.loadCalibration()) {
    Serial.println("Current calibration loaded:");
    cal.printSavedCalibration();
    printCalibration(cal.mag_softiron, cal.mag_hardiron, cal.mag_field, cal.gyro_zerorate, cal.accel_zerog, "read at startup");
 
  }
  else
  {
    Serial.println("calibration load failed.");
  }
  Serial.println("\n\nWaiting for data from MotionCal software...");
  
#endif
}

void performEEPROMTest()
{
  Serial.println("Calibration filesys test");

  Serial.print("Has EEPROM: "); Serial.println(cal.hasEEPROM());
  Serial.print("Has FLASH: "); Serial.println(cal.hasFLASH());

  populateTestValues();


  if (! cal.loadCalibration()) {
    Serial.println("No calibration loaded/found... will start with defaults");
  } else {
    Serial.println("Loaded existing calibration");
  }

  // in uTesla
  for(int i = 0; i < 3; i++)
  {
    cal.mag_hardiron[i] = hardIronTestValues[i];
  }
  // in uTesla
  for(int i = 0; i < 9; i++)
  {
    cal.mag_softiron[i] = softIronTestValues[i];
  }

  // Earth total magnetic field strength in uTesla (dependent on location and time of the year),
  // visit: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm)
  cal.mag_field = magneticField; // approximate value for locations along the equator

  // in Radians/s
  for(int i = 0; i < 3; i++)
  {
    cal.gyro_zerorate[i] = gyroZeroRotate[i];
  }

  if (! cal.saveCalibration()) {
    Serial.println("**WARNING** Couldn't save calibration");
  } else {
    Serial.println("Wrote calibration");    
  }

  cal.printSavedCalibration();
  Serial.println("\n\n....\n\n");

  readAndPrintCalibration();

  testCalibration();
}

void populateTestValues()
{
  hardIronTestValues[0] = -3.35;
  hardIronTestValues[1] = -0.74;
  hardIronTestValues[2] = -40.79;

  magneticField = 45.0;

  softIronTestValues[0] = 0.965;
  softIronTestValues[1] = 0.018;
  softIronTestValues[2] = 0.010;  
  softIronTestValues[3] = 0.018;
  softIronTestValues[4] = 0.960;
  softIronTestValues[5] = 0.003;  
  softIronTestValues[6] = 0.010;
  softIronTestValues[7] = 0.003;
  softIronTestValues[8] = 1.080;

  gyroZeroRotate[0] = 0.05;
  gyroZeroRotate[1] = -0.01;
  gyroZeroRotate[2] = -0.01;
}

void readAndPrintCalibration()
{

  if (! cal.loadCalibration()) {
    Serial.println("**WARNING** No calibration loaded/found");
  }
  cal.printSavedCalibration();

  printCalibration(cal.mag_softiron, cal.mag_hardiron, cal.mag_field, cal.gyro_zerorate, cal.accel_zerog, "read from readAndPrintCalibration");
 
  /* Serial.println("Calibrations found: ");
  Serial.print("\tMagnetic Hard Offset: ");
  for (int i=0; i<3; i++) {
    Serial.print(cal.mag_hardiron[i]); 
    if (i != 2) Serial.print(", ");
  }
  Serial.println();
  
  Serial.print("\tMagnetic Soft Offset: ");
  for (int i=0; i<9; i++) {
    Serial.print(cal.mag_softiron[i]); 
    if (i != 8) Serial.print(", ");
  }
  Serial.println();

  Serial.print("\tMagnetic Field Magnitude: ");
  Serial.println(cal.mag_field);

  Serial.print("\tGyro Zero Rate Offset: ");
  for (int i=0; i<3; i++) {
    Serial.print(cal.gyro_zerorate[i]); 
    if (i != 2) Serial.print(", ");
  }
  Serial.println();

  Serial.print("\tAccel Zero G Offset: ");
  for (int i=0; i<3; i++) {
    Serial.print(cal.accel_zerog[i]); 
    if (i != 2) Serial.print(", ");
  }
  Serial.println();*/
}

void printCalibration(float magnetometerSoftIron[], float magnetometerHardIron[], float magneticField, float gyroZeroRotate[], float accelZeroG[], char prefix[])
{
  Serial.printf("'%s' Calibrations found: \n", prefix);
  Serial.print("\tMagnetic Hard Offset: ");
  if (magnetometerHardIron == NULL)
  {
    Serial.print("NULL");
  }
  else
  {
    for (int i=0; i<3; i++) {
      Serial.print(magnetometerHardIron[i], 4); 
      if (i != 2) Serial.print(", ");
    }
  }
  Serial.println();
  
  Serial.print("\tMagnetic Soft Offset: ");
  if (magnetometerSoftIron == NULL)
  {
    Serial.print("NULL");
  }
  else
  {
    for (int i=0; i<9; i++) {
      Serial.print(magnetometerSoftIron[i], 4); 
      if (i != 8) Serial.print(", ");
    }
  }
  Serial.println();

  Serial.print("\tMagnetic Field Magnitude: ");
  Serial.println(magneticField);

  Serial.print("\tGyro Zero Rate Offset: ");
  if (gyroZeroRotate == NULL)
  {
    Serial.print("NULL");
  }
  else
  {
    for (int i=0; i<3; i++) 
    {
      Serial.print(gyroZeroRotate[i], 4); 
      if (i != 2) Serial.print(", ");
    }
  }
  Serial.println();

  Serial.print("\tAccel Zero G Offset: ");
  if (accelZeroG == NULL)
  {
    Serial.print("NULL");
  }
  else
  {
    for (int i=0; i<3; i++) {
      Serial.print(accelZeroG[i], 4); 
      if (i != 2) Serial.print(", ");
    }
  }
  Serial.println();
} 


void testCalibration()
{
  bool hardIronCalibrationPassed = true;
  bool softIronCalibrationPassed = true;
  bool gyroCalibrationPassed = true;
  bool magneticCalibrationPassed = true;
 
  Serial.print("hardIron: ");
  for (int i=0; i<3; i++) {
    Serial.printf("%F3 [%s] ", cal.mag_hardiron[i], cal.mag_hardiron[i] == hardIronTestValues[i]? "Y":"N");
    if (i != 2) Serial.print(", ");
    if (cal.mag_hardiron[i] != hardIronTestValues[i]) hardIronCalibrationPassed = false;
  }

  Serial.println();
  Serial.print("softIron: ");
  for (int i=0; i<9; i++) {
    Serial.printf("%F3 [%s] ", cal.mag_softiron[i], cal.mag_softiron[i] == softIronTestValues[i]? "Y":"N");
    if (i != 8) Serial.print(", ");
    if (cal.mag_softiron[i] != softIronTestValues[i]) softIronCalibrationPassed = false;
  }

  Serial.println();
  Serial.print("gyroscope: ");
  for (int i=0; i<3; i++) {
    Serial.printf("%F3 [%s] ", cal.gyro_zerorate[i], cal.gyro_zerorate[i] == gyroZeroRotate[i]? "Y":"N");
    if (i != 2) Serial.print(", ");
    if (cal.gyro_zerorate[i] != gyroZeroRotate[i]) gyroCalibrationPassed = false;
  }
  Serial.println();
  Serial.printf("magnetic: %F3 [%s] \n", cal.mag_field, cal.mag_field == magneticField? "Y":"N");
  if (cal.mag_field != magneticField) magneticCalibrationPassed = false;
  if (hardIronCalibrationPassed && softIronCalibrationPassed && gyroCalibrationPassed && magneticCalibrationPassed)
    Serial.println("All calibration tests passed");
  else
  {
   Serial.printf("hardIron Calibration: %s\n", hardIronCalibrationPassed? "Pass": "Fail"); 
   Serial.printf("softIron Calibration: %s\n", softIronCalibrationPassed? "Pass": "Fail"); 
   Serial.printf("Gryroscope Calibration: %s\n", gyroCalibrationPassed? "Pass": "Fail"); 
   Serial.printf("Magnetic Field Calibrationn: %s\n", magneticCalibrationPassed? "Pass": "Fail"); 
  }
}




void loop() {
  receiveCalibration();
}

// Standard CRC16-CCITT for MotionCal verification
uint16_t _crc16_update(uint16_t crc, uint8_t a) {
  int i;
  crc ^= a << 8;
  for (i = 0; i < 8; i++) {
    if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
    else crc <<= 1;
  }
  return crc;
}

void receiveCalibration() { 
  uint16_t crc;   
  byte b, i; 

  if (Serial.available()) { 
    b = Serial.read(); 

    // Look for 'uT' header
    if (calcount == 0 && b != 117) return; 
    if (calcount == 1 && b != 84) { 
      calcount = 0; 
      return; 
    } 
    
    caldata[calcount++] = b; 

    if (calcount < calibrationDataSize) return; 

    // Verify CRC
    crc = 0xFFFF; 
    for (i=0; i < calibrationDataSize; i++) { 
      crc = _crc16_update(crc, caldata[i]); 
    } 

    if (crc == 0) { 
      Serial.println("\nValid MotionCal packet received!");
      processAndSave(caldata);
      calcount = 0; 
      return; 
    } 

    // If CRC fails, look for next header in buffer
    for (i=2; i < calibrationDataSize; i++) { 
      if (caldata[i] == 117 && caldata[i+1] == 84) { 
        calcount = calibrationDataSize - i; 
        memmove(caldata, caldata + i, calcount); 
        return; 
      } 
    } 
    calcount = 0; 
  } 
}

void processAndSave(byte* data) {
  MotionCalPacket* packet = (MotionCalPacket*)data;

  // 1. Map Magnetometer Hard Iron (Indices 6, 7, 8)
  cal.mag_hardiron[0] = packet->offsets[6];
  cal.mag_hardiron[1] = packet->offsets[7];
  cal.mag_hardiron[2] = packet->offsets[8];

  // 2. Map Magnetometer Soft Iron (Full 3x3 Matrix)
  for (int i = 0; i < 9; i++) {
    cal.mag_softiron[i] = packet->softiron[i];
  }

  // 3. Map Gyro Zero Rate (Indices 3, 4, 5)
  cal.gyro_zerorate[0] = packet->offsets[3];
  cal.gyro_zerorate[1] = packet->offsets[4];
  cal.gyro_zerorate[2] = packet->offsets[5];

  // 4. Map Field Strength
  cal.mag_field = packet->field;
  printCalibration(cal.mag_softiron, cal.mag_hardiron, cal.mag_field, cal.gyro_zerorate, NULL, "read from motion cal");
  // 5. Save to ESP32 Flash
  if (cal.saveCalibration()) {
    Serial.println("Wrote calibration to Flash/NVS");
    cal.printSavedCalibration();
  } else {
    Serial.println("**WARNING** Couldn't save calibration");
  }
}


