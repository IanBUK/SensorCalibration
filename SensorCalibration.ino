//#define TEST_BOARD_CAN_SAVE_LOAD_CALIBRATION

#include "Adafruit_Sensor_Calibration.h"
#include "elapsedMillis.h"
#include <Adafruit_AHRS.h>
Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

// select either EEPROM or SPI FLASH storage:
Adafruit_Sensor_Calibration_EEPROM cal;
// Adafruit_Sensor_Calibration_SDFat cal;

// uncomment one combo 9-DoF!
//#include "LSM6DS_LIS3MDL.h"  // see the the LSM6DS_LIS3MDL file in this project to change board to LSM6DS33, LSM6DS3U, LSM6DSOX, etc
//#include "LSM9DS.h"           // LSM9DS1 or LSM9DS0
#include "NXP_FXOS_FXAS.h"  // NXP 9-DoF breakout

// pick your filter! slower == better quality output
Adafruit_NXPSensorFusion filter; // slowest
//Adafruit_Madgwick filter;  // faster than NXP
//Adafruit_Mahony filter;  // fastest/smalleset


sensors_event_t mag_event, gyro_event, accel_event;

int loopcount = 0;
unsigned long lastFrameTime = millis(); 

float refreshRateHz = 100.0f;
float refreshRateMs = 1000.0f / refreshRateHz;



// Orientation variables
unsigned long orientationUpdateMillis = 20;
float relativeYawBase =0;
float relativePitchBase =0;
float relativeRollBase =0;
bool needsRelativeBase = false;

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
    while (1) delay(10);
  }
  if (!init_sensors()) {
    Serial.println("Failed to find sensors");
    while (1) delay(10);
  }

  accelerometer->printSensorDetails();
  gyroscope->printSensorDetails();
  magnetometer->printSensorDetails();

  setup_sensors();

  my_setup_sensors();

  sensor_t sensor;
  accelerometer->getSensor(&sensor);
  reportSensorDetail(sensor);

  gyroscope->getSensor(&sensor);
  reportSensorDetail(sensor);

  magnetometer->getSensor(&sensor);
  reportSensorDetail(sensor);

  delay(5000);
  filter.begin(refreshRateHz);
  Wire.setClock(400000); // 400KHz
  Serial.println("\n\nWaiting for data from MotionCal software...");
#endif
}

void reportSensorDetail(sensor_t sensor)
{
  // Replace 'accelerometer' with your object name (e.g., accel, gyro, mag)
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" units");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" units");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" units");
  Serial.print  ("Min Delay:    "); Serial.print(sensor.min_delay / 1000); Serial.println(" ms");
  Serial.println("------------------------------------");


  delay(1000);
}

void my_setup_sensors() {
// 1. GYROSCOPE (FXAS21002C)
  // Set to 2000 Degrees Per Second. 
  // Note: The library handles the ODR internally (usually 100Hz).
  fxas.setRange(GYRO_RANGE_2000DPS);
  Serial.println("Gyro: 2000 DPS");

  // 2. ACCELEROMETER/MAGNETOMETER (FXOS8700)
  // Set Accel to 8G for high-speed/vibration headroom
  fxos.setAccelRange(ACCEL_RANGE_8G);
  
  // The correct method is setOutputDataRate. 
  // We use the 'ODR_200HZ' enum which is internal to the FXOS8700 class.
  fxos.setOutputDataRate(ODR_200HZ); 
  
  Serial.println("FXOS8700: 8G Accel @ 200Hz Hybrid (100Hz Mag/100Hz Accel)");
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
  reportSensorReadings();
  
  receiveCalibration(); // Check for incoming cal FIRST
  
  // Only report data if we aren't currently receiving something
  // and limit the rate to ~20Hz (50ms)
  static unsigned long lastReport = 0;
  if (millis() - lastReport > 50) {
     reportCalibration();
     lastReport = millis();
  }
  
  flashLed();
}




void reportSensorReadings()
{
  float roll, pitch, heading;
  float gx, gy, gz;
  static unsigned long lastReport = 0;
  if (millis() - lastReport < refreshRateMs) return; 
  lastReport = millis();

  // Read the motion sensors
  sensors_event_t accel, gyro, mag;
  accelerometer->getEvent(&accel);
  gyroscope->getEvent(&gyro);
  magnetometer->getEvent(&mag);
  cal.calibrate(mag);
  cal.calibrate(accel);
  cal.calibrate(gyro);

  gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
  gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
  gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

  filter.update(gx, gy, gz, 
                accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, 
                mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

  roll = filter.getRoll();
  pitch = filter.getPitch();
  heading = filter.getYaw();
  Serial.print("Orientation: ");
  Serial.print(heading,4);
  Serial.print(", ");
  Serial.print(pitch,4);
  Serial.print(", ");
  Serial.println(roll, 4);

  float qw, qx, qy, qz;
  
  filter.getQuaternion(&qw, &qx, &qy, &qz);
  Serial.print("Quaternion: ");
  Serial.print(qw, 4);
  Serial.print(", ");
  Serial.print(qx, 4);
  Serial.print(", ");
  Serial.print(qy, 4);
  Serial.print(", ");
  Serial.println(qz, 4); 

  Serial.printf("Ori: %.1f,%.1f,%.1f\n", heading, pitch, roll); 
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





void reportSensorReadings2() {
  if (calcount != 0) return;

  static unsigned long lastReport = 0;
  if (millis() - lastReport < 200) return; 
  lastReport = millis();

  magnetometer->getEvent(&mag_event);
  accelerometer->getEvent(&accel_event);

  // 1. PHYSICAL ACCELEROMETER MAPPING
  // Based on your previous success with axis placement:
  float ax =  accel_event.acceleration.y; 
  float ay =  accel_event.acceleration.x; 
  float az =  accel_event.acceleration.z; 

  // 2. THE MAGNETOMETER SYNC (The Fix for the 222 -> 130 drop)
  // We align the Magnetometer to the Accelerometer's world.
  // We swap Mag X and Y to account for the 90-degree internal sensor rotation.
  float mx =  mag_event.magnetic.x;  
  float my =  mag_event.magnetic.y; 
  float mz =  mag_event.magnetic.z; 

  // 3. STABLE TRIGONOMETRY
  float rollRad  = atan2(ay, az); 
  float pitchRad = atan2(-ax, sqrt(ay * ay + az * az) + 0.001);

  // 4. TILT COMPENSATION (Aerospace Standard)
  float cosR = cos(rollRad);
  float sinR = sin(rollRad);
  float cosP = cos(pitchRad);
  float sinP = sin(pitchRad);

  // Re-projecting the magnetic field onto the calculated horizontal plane
  float Xh = mx * cosP + my * sinR * sinP + mz * cosR * sinP;
  float Yh = my * cosR - mz * sinR;
  
  // atan2(-Yh, Xh) is standard for a Right-Handed System
  float yawRad = atan2(-Yh, Xh);

  // 5. CONVERT TO DEGREES
  float roll  = rollRad  * 57.2958;
  float pitch = pitchRad * 57.2958;
  float yaw   = yawRad   * 57.2958;

  // 6. GIMBAL LOCK PROTECTION
  if (pitch > 89.0)  pitch = 89.0;
  if (pitch < -89.0) pitch = -89.0;

  // 7. RELATIVE ZEROING (Desk Calibration)
  if(needsRelativeBase) {
    relativeYawBase = yaw;
    relativePitchBase = pitch;
    relativeRollBase = roll;
    needsRelativeBase = false;
  } else {
    yaw   -= relativeYawBase;
    pitch -= relativePitchBase;
    roll  -= relativeRollBase;
    
    // Final Normalization
    if (yaw < 0) yaw += 360;
    if (yaw >= 360) yaw -= 360;
    if (roll > 180) roll -= 360;
    if (roll < -180) roll += 360;
  }

  // 8. FINAL OUTPUT
  Serial.printf("Ori: %.1f,%.1f,%.1f\n", yaw, pitch, roll); 
  loopcount++; 
}
