//#define TEST_BOARD_CAN_SAVE_LOAD_CALIBRATION

#include "Adafruit_Sensor_Calibration.h"
#include "elapsedMillis.h"

Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

// select either EEPROM or SPI FLASH storage:
Adafruit_Sensor_Calibration_EEPROM cal;
// Adafruit_Sensor_Calibration_SDFat cal;

// uncomment one combo 9-DoF!
//#include "LSM6DS_LIS3MDL.h"  // see the the LSM6DS_LIS3MDL file in this project to change board to LSM6DS33, LSM6DS3U, LSM6DSOX, etc
//#include "LSM9DS.h"           // LSM9DS1 or LSM9DS0
#include "NXP_FXOS_FXAS.h"  // NXP 9-DoF breakout

sensors_event_t mag_event, gyro_event, accel_event;

int loopcount = 0;
unsigned long lastFrameTime = millis(); 

// Orientation variables
unsigned long orientationUpdateMillis = 20;
float orientationXDelta = 1; 
float orientationYDelta = 2; 
float orientationZDelta = 3; 
float orientationX; 
float orientationY; 
float orientationZ; 

// LED Variables
const int ledPin = 13; 
int ledState = LOW; 
int ledFastblinks = 0; 
elapsedMillis ledMillis = 0; 

#ifdef TEST_BOARD_CAN_SAVE_LOAD_CALIBRATION
// calibration variables
float hardIronTestValues[3];
float softIronTestValues[9];
float magneticField = 45.0;
float gyroZeroRotate[3];
#endif

// MotionCal uses an 68-byte packet format
const int calibrationDataSize = 68;
byte caldata[calibrationDataSize];
byte calcount = 0;

// Internal struct to unpack the MotionCal binary stream
struct __attribute__((packed)) MotionCalPacket {
  uint16_t header;        // 'uT' (117, 84)
  float accel_offsets[3]; // PC sends 0.0 here
  float gyro_offsets[3];  // PC sends 0.0 here
  float mag_offsets[3];   // PC sends magcal.V[i]
  float mag_field;        // PC sends magcal.B
  float soft_iron[6];     // PC sends 6 elements of invW
  uint16_t crc;           // 2-byte CRC (Little-Endian)
};

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  delay(100);
  Serial.println("device started");
  if (!cal.begin()) {
    Serial.println("Failed to initialize calibration EEPROM");
    while (1) yield();
  }

#ifdef TEST_BOARD_CAN_SAVE_LOAD_CALIBRATION
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
  if (!init_sensors()) {
    Serial.println("Failed to find sensors");
    while (1) delay(10);
  }

  Serial.println("\n\nWaiting for data from MotionCal software...");
#endif
}

#ifdef TEST_BOARD_CAN_SAVE_LOAD_CALIBRATION
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
#endif

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


void loop() {
  receiveCalibration(); // Check for incoming cal FIRST
  
  // Only report data if we aren't currently receiving something
  // and limit the rate to ~20Hz (50ms)
  static unsigned long lastReport = 0;
  if (millis() - lastReport > 50) {
     reportSensorReadings();
     reportCalibration();
     lastReport = millis();
  }
  
  flashLed();
}

void reportSensorReadings() {
  // SILENCE: Don't talk if we are in the middle of receiving a packet
  if (calcount != 0) return;

  static unsigned long lastReport = 0;
  if (millis() - lastReport < 40) return; // Limit to 25Hz to keep Serial clean
  lastReport = millis();

  if (magnetometer == NULL || gyroscope == NULL || accelerometer == NULL) return;

  // Get fresh sensor events
  magnetometer->getEvent(&mag_event);
  gyroscope->getEvent(&gyro_event);
  accelerometer->getEvent(&accel_event);

  // 1. Calculate Roll and Pitch from Accelerometer
  // Roll = atan2(Y, Z), Pitch = atan2(-X, sqrt(Y^2 + Z^2))
  float roll  = atan2(accel_event.acceleration.y, accel_event.acceleration.z) * 57.2958;
  float pitch = atan2(-accel_event.acceleration.x, 
                sqrt(accel_event.acceleration.y * accel_event.acceleration.y + 
                     accel_event.acceleration.z * accel_event.acceleration.z)) * 57.2958;

  // 2. Calculate Yaw (Heading) from Magnetometer
  // Note: This is a "tilt-compensated" simplification
  float mag_x = mag_event.magnetic.x;
  float mag_y = mag_event.magnetic.y;
  float yaw = atan2(mag_y, mag_x) * 57.2958;
  
  // Normalize yaw to 0-360
  if (yaw < 0) yaw += 360;

  // 3. Report to MotionCal (Required format)
  Serial.print("Raw: ");
  Serial.print(int(accel_event.acceleration.x*8192/9.8)); Serial.print(",");
  Serial.print(int(accel_event.acceleration.y*8192/9.8)); Serial.print(",");
  Serial.print(int(accel_event.acceleration.z*8192/9.8)); Serial.print(",");
  Serial.print(int(gyro_event.gyro.x*SENSORS_RADS_TO_DPS*16)); Serial.print(",");
  Serial.print(int(gyro_event.gyro.y*SENSORS_RADS_TO_DPS*16)); Serial.print(",");
  Serial.print(int(gyro_event.gyro.z*SENSORS_RADS_TO_DPS*16)); Serial.print(",");
  Serial.print(int(mag_event.magnetic.x*10)); Serial.print(",");
  Serial.print(int(mag_event.magnetic.y*10)); Serial.print(",");
  Serial.print(int(mag_event.magnetic.z*10)); Serial.println("");

  // 4. Report Roll, Pitch, Yaw for your own debugging
  // We use "RPY:" so it doesn't confuse MotionCal's "Ori:" or "Raw:" parsers
  //Serial.printf("Ori: %0.2f, %0.2f, %0.2f\n", roll, pitch, yaw);

  Serial.print("Ori: "); 
  Serial.print(roll); 
  Serial.print(","); 
  Serial.print(pitch); 
  Serial.print(","); 
  Serial.print(yaw); 
  Serial.print("\n"); 

  // unified data
  Serial.print("Uni: ");
  Serial.print(accel_event.acceleration.x); Serial.print(",");
  Serial.print(accel_event.acceleration.y); Serial.print(",");
  Serial.print(accel_event.acceleration.z); Serial.print(",");
  Serial.print(gyro_event.gyro.x, 4); Serial.print(",");
  Serial.print(gyro_event.gyro.y, 4); Serial.print(",");
  Serial.print(gyro_event.gyro.z, 4); Serial.print(",");
  Serial.print(mag_event.magnetic.x); Serial.print(",");
  Serial.print(mag_event.magnetic.y); Serial.print(",");
  Serial.print(mag_event.magnetic.z); Serial.println("");
    
  loopcount = loopcount + 1; 
}


void reportSensorReadings2()
{
  int ax, ay, az; 
  int gx, gy, gz; 
  int mx, my, mz; 

  if (calcount != 0) return;
  
  unsigned long now = millis();

  if ((now - lastFrameTime) > orientationUpdateMillis) 
  { 
    // get orientation from sensor

    // test data
    orientationX += orientationXDelta; 
    orientationY += orientationYDelta; 
    orientationZ += orientationZDelta; 

    if (orientationX > 360.0) orientationX -= 360.0; 
    if (orientationX < 0) orientationX += 360.0; 

    if (orientationY > 360.0) orientationY -= 360.0; 
    if (orientationY < 0) orientationY += 360.0; 

    if (orientationZ > 360.0) orientationZ -= 360.0; 
    if (orientationZ < 0) orientationZ += 360.0; 




    lastFrameTime = now; 

    Serial.print("Ori: "); 
    Serial.print(orientationX); 
    Serial.print(","); 
    Serial.print(orientationY); 
    Serial.print(","); 
    Serial.print(orientationZ); 
    Serial.print("\n"); 
  } 

  if (magnetometer == NULL || gyroscope == NULL || accelerometer == NULL)
    return;

  // get and print uncalibrated data 
  magnetometer->getEvent(&mag_event);
  gyroscope->getEvent(&gyro_event);
  accelerometer->getEvent(&accel_event);
  
  // 'Raw' values to match expectation of MotionCal
  Serial.print("Raw: ");
  Serial.print(int(accel_event.acceleration.x*8192/9.8)); Serial.print(",");
  Serial.print(int(accel_event.acceleration.y*8192/9.8)); Serial.print(",");
  Serial.print(int(accel_event.acceleration.z*8192/9.8)); Serial.print(",");
  Serial.print(int(gyro_event.gyro.x*SENSORS_RADS_TO_DPS*16)); Serial.print(",");
  Serial.print(int(gyro_event.gyro.y*SENSORS_RADS_TO_DPS*16)); Serial.print(",");
  Serial.print(int(gyro_event.gyro.z*SENSORS_RADS_TO_DPS*16)); Serial.print(",");
  Serial.print(int(mag_event.magnetic.x*10)); Serial.print(",");
  Serial.print(int(mag_event.magnetic.y*10)); Serial.print(",");
  Serial.print(int(mag_event.magnetic.z*10)); Serial.println("");

  // unified data
  Serial.print("Uni: ");
  Serial.print(accel_event.acceleration.x); Serial.print(",");
  Serial.print(accel_event.acceleration.y); Serial.print(",");
  Serial.print(accel_event.acceleration.z); Serial.print(",");
  Serial.print(gyro_event.gyro.x, 4); Serial.print(",");
  Serial.print(gyro_event.gyro.y, 4); Serial.print(",");
  Serial.print(gyro_event.gyro.z, 4); Serial.print(",");
  Serial.print(mag_event.magnetic.x); Serial.print(",");
  Serial.print(mag_event.magnetic.y); Serial.print(",");
  Serial.print(mag_event.magnetic.z); Serial.println("");
    
  loopcount = loopcount + 1; 

}

void reportCalibration()
{
  if (calcount != 0) return;

  // occasionally print calibration 
  if (loopcount == 50 || loopcount > 100) 
  { 
    Serial.print("Cal1:");
    for (int i=0; i<3; i++) {
      Serial.print(cal.accel_zerog[i], 3); 
      Serial.print(",");
    }
    for (int i=0; i<3; i++) {
      Serial.print(cal.gyro_zerorate[i], 3);
      Serial.print(",");
    }  
    for (int i=0; i<3; i++) {
      Serial.print(cal.mag_hardiron[i], 3); 
      Serial.print(",");
    }  
    Serial.println(cal.mag_field, 3);
    loopcount++;
  } 

  if (loopcount >= 100) 
  { 
    Serial.print("Cal2:");
    for (int i=0; i<9; i++) {
      Serial.print(cal.mag_softiron[i], 4); 
      if (i < 8) Serial.print(',');
    }
    Serial.println();
    loopcount = 0;
  } 

  delay(10);
}

void flashLed()
{
    // blink LED, slow normally, fast when calibration written 
  if (ledMillis >= 1000) { 
    if (ledFastblinks > 0) { 
      ledFastblinks = ledFastblinks - 1; 
      ledMillis -= 125; 
    } else { 
      ledMillis -= 1000; 
    } 

    ledState = (ledState == LOW) ? HIGH : LOW;
    digitalWrite(ledPin, ledState); 
  } 
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
  while (Serial.available()) {
    byte b = Serial.read();

    if (calcount == 0) {
      if (b == 117) caldata[calcount++] = b;
      continue;
    }
    if (calcount == 1) {
      if (b == 84) caldata[calcount++] = b;
      else calcount = 0;
      continue;
    }

    caldata[calcount++] = b;

    if (calcount >= 68) {
      // Calculate CRC on first 66 bytes
      uint16_t crc = 0xFFFF;
      for (int i = 0; i < 66; i++) {
        crc = _crc16_update(crc, caldata[i]);
      }

      // PC sends CRC Little-Endian: buf[66] is low byte, buf[67] is high byte
      uint16_t sentCrc = caldata[66] | (caldata[67] << 8);

      if (crc == sentCrc) {
        Serial.println("CalR: SUCCESS! CRC Match.");
        processAndSave(caldata);
      } else {
        Serial.printf("CalR: CRC Mismatch. Calc: 0x%04X, Sent: 0x%04X\n", crc, sentCrc);
        // Emergency Save: If the field strength is reasonable, save anyway
        float fieldCheck;
        memcpy(&fieldCheck, &caldata[2 + (9 * 4)], 4); 
        if (fieldCheck > 20.0 && fieldCheck < 80.0) {
           Serial.println("CalR: Data looks valid despite CRC. Processing...");
           processAndSave(caldata);
        }
      }
      calcount = 0; 
    }
  }
}




void receiveCalibration4() {
  while (Serial.available()) {
    byte b = Serial.read();

    // Sync to 'uT' header
    if (calcount == 0) {
      if (b == 117) caldata[calcount++] = b;
      continue;
    }
    if (calcount == 1) {
      if (b == 84) caldata[calcount++] = b;
      else calcount = 0;
      continue;
    }

    caldata[calcount++] = b;

    if (calcount >= 68) {
      Serial.println("\n--- NEW CALIBRATION PACKET RECEIVED ---");
      
      // 1. Report Floats for Comparison
      for (int i = 0; i < 16; i++) {
        float val;
        memcpy(&val, &caldata[2 + (i * 4)], 4);
        
        // Labeling based on MotionCal's standard float array order
        const char* label = "";
        if (i >= 0 && i <= 2) label = "(Accel Offset)";
        if (i >= 3 && i <= 5) label = "(Gyro Offset)";
        if (i >= 6 && i <= 8) label = "(Mag Hard-Iron)";
        if (i >= 9 && i <= 11) label = "(Soft-Iron Row 1)";
        if (i >= 12 && i <= 14) label = "(Soft-Iron Row 2/3)"; // Mapping varies by library
        if (i == 15) label = "(Mag Field Strength)";

        Serial.printf("Float [%02d]: %10.6f %s\n", i, val, label);
      }

      // 2. Perform the save
      // We are trusting the 'uT' header + 68 byte length for now
      processAndSave(caldata);
      
      Serial.println("---------------------------------------\n");
      calcount = 0; 
    }
  }
}

void receiveCalibration3() {
  while (Serial.available()) {
    byte b = Serial.read();

    // Look for 'u' (117)
    if (calcount == 0) {
      if (b == 117) caldata[calcount++] = b;
      continue;
    }

    // Look for 'T' (84)
    if (calcount == 1) {
      if (b == 84) {
        caldata[calcount++] = b;
      } else {
        calcount = 0; // Reset if not 'uT'
      }
      continue;
    }

    // Fill the rest of the 68 bytes
    caldata[calcount++] = b;

    if (calcount >= calibrationDataSize) {
      uint16_t crc = 0xFFFF;
      for (int i = 0; i < calibrationDataSize; i++) {
        crc = _crc16_update(crc, caldata[i]);
      }

      if (crc == 0) {
        Serial.println("CalR: SUCCESS! Valid packet.");
        processAndSave(caldata);
      } else {
    Serial.printf("\n--- CALIBRATION CRC FAIL (0x%04X) ---\n", crc);
        
        // 1. Raw Hex Dump
        Serial.print("HEX: ");
        for(int i=0; i < 68; i++) {
          Serial.printf("%02X ", caldata[i]);
        }
        Serial.println();

        // 2. Interpret 64 bytes of payload as 16 Floats
        // Data starts at index 2, ends at index 65
        Serial.println("FLOAT DATA IN PACKET:");
        for (int i = 0; i < 16; i++) {
          float val;
          int startIndex = 2 + (i * 4);
          memcpy(&val, &caldata[startIndex], 4);
          
          Serial.printf("[%02d]: %f  ", i, val);
          if ((i + 1) % 4 == 0) Serial.println(); // New line every 4 floats
        }
        
        uint16_t receivedCrc = (caldata[66] << 8) | caldata[67];
        Serial.printf("Tail CRC Bytes: 0x%04X\n", receivedCrc);
        Serial.println("--------------------------------------\n");
      }
      calcount = 0; // Always reset
    }
  }
}
void receiveCalibration2() { 
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

    if (calcount < calibrationDataSize) 
    {
      //Serial.printf("%d bytes received, %d expected\n", calcount, calibrationDataSize);
      return; 
    }
    Serial.printf("CalR:process message.   \n");

    // Verify CRC
    crc = 0xFFFF; 
    for (i=0; i < calibrationDataSize; i++) { 
      crc = _crc16_update(crc, caldata[i]); 
    } 

    if (crc == 0) { 
      Serial.println("CalR:Valid MotionCal packet received!");
      processAndSave(caldata);
      calcount = 0; 
      return; 
    } 
    Serial.printf("CalR:Invalid MotionCal packeyt received: crc: 0x%o4X\n", crc);

    // If CRC fails, look for next header in buffer
    for (i=2; i < calibrationDataSize; i++) { 
      if (caldata[i] == 117 && caldata[i+1] == 84) { 
        calcount = calibrationDataSize - i; 
        memmove(caldata, caldata + i, calcount); 
        Serial.printf("CalR:CRC failed, looking for next header in buffer, found at calCount: %d\'n", calcount);
        return; 
      } 
    } 
    Serial.println("CalR:CRC failed, couldn't find header in buffer");
    calcount = 0; 
  } 
}

void processAndSave(byte* data) {
  MotionCalPacket* packet = (MotionCalPacket*)data;

  // 1. Reconstruct the 3x3 Soft Iron Matrix from the 6 symmetric elements
  // Row 0: [0,0], [0,1], [0,2]
  cal.mag_softiron[0] = packet->soft_iron[0]; 
  cal.mag_softiron[1] = packet->soft_iron[3]; 
  cal.mag_softiron[2] = packet->soft_iron[4]; 
  // Row 1: [1,0], [1,1], [1,2]
  cal.mag_softiron[3] = packet->soft_iron[3]; 
  cal.mag_softiron[4] = packet->soft_iron[1]; 
  cal.mag_softiron[5] = packet->soft_iron[5]; 
  // Row 2: [2,0], [2,1], [2,2]
  cal.mag_softiron[6] = packet->soft_iron[4]; 
  cal.mag_softiron[7] = packet->soft_iron[5]; 
  cal.mag_softiron[8] = packet->soft_iron[2]; 

  // 2. Map Hard Iron and Field Strength
  for(int i=0; i<3; i++) cal.mag_hardiron[i] = packet->mag_offsets[i];
  cal.mag_field = packet->mag_field;

  // --- THE TRUE TEST: FINAL DATA REPORT ---
  Serial.println("\n========================================");
  Serial.println("   ESP32 CALIBRATION DATA RECEIVED      ");
  Serial.println("========================================");
  
  Serial.printf("Hard Iron (Offsets): %7.2f, %7.2f, %7.2f\n", 
                cal.mag_hardiron[0], cal.mag_hardiron[1], cal.mag_hardiron[2]);
  
  Serial.printf("Field Strength:      %7.2f uT\n", cal.mag_field);
  
  Serial.println("\nSoft Iron Matrix (3x3):");
  for (int i=0; i<3; i++) {
    Serial.printf("  [Row %d]: %8.4f, %8.4f, %8.4f\n", 
                  i, cal.mag_softiron[i*3], cal.mag_softiron[i*3+1], cal.mag_softiron[i*3+2]);
  }
  
  // Quick sanity check for the user
  if (cal.mag_field < 10.0) {
    Serial.println("\n!! WARNING: Field strength looks too low for the UK !!");
  }
  Serial.println("========================================\n");

  Serial.println("call cal.saveCalibration");
  // 3. Save to Flash
  if (cal.saveCalibration()) {
    Serial.println("SUCCESS: Written to Flash/NVS.");
    cal.printSavedCalibration();
    Serial.println("done");
  } else {
    Serial.println("ERROR: Flash write failed.");
  }
}

// void processAndSave2(byte* data) {
//   MotionCalPacket* packet = (MotionCalPacket*)data;

//   // 1. Map Magnetometer Hard Iron (Indices 6, 7, 8)
//   cal.mag_hardiron[0] = packet->offsets[6];
//   cal.mag_hardiron[1] = packet->offsets[7];
//   cal.mag_hardiron[2] = packet->offsets[8];

//   // 2. Map Magnetometer Soft Iron (Full 3x3 Matrix)
//   for (int i = 0; i < 9; i++) {
//     cal.mag_softiron[i] = packet->softiron[i];
//   }

//   // 3. Map Gyro Zero Rate (Indices 3, 4, 5)
//   cal.gyro_zerorate[0] = packet->offsets[3];
//   cal.gyro_zerorate[1] = packet->offsets[4];
//   cal.gyro_zerorate[2] = packet->offsets[5];

//   // 4. Map Field Strength
//   cal.mag_field = packet->field;
//   printCalibration(cal.mag_softiron, cal.mag_hardiron, cal.mag_field, cal.gyro_zerorate, NULL, "read from motion cal");
//   // 5. Save to ESP32 Flash
//   if (cal.saveCalibration()) {
//     Serial.println("Wrote calibration to Flash/NVS");
//     cal.printSavedCalibration();
//   } else {
//     Serial.println("**WARNING** Couldn't save calibration");
//   }
// }



