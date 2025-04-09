/* 
 * Main Mission Code V2.0 - Updated with u-blox GPS Library
 * Environmental Sensor Array with I2C Interface
 */

#include <Wire.h>
#include <RTClib.h>
#include <Multichannel_Gas_GMXXX.h>
#include <HP20x_dev.h>
#include "DFRobot_OzoneSensor.h"
#include <LTR390.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> // Updated GPS library
#include <AM2302-Sensor.h>
#include <ArduCAM.h>
#include <SPI.h>
#include <SD.h>
#include "memorysaver.h"

// Define constants
#define I2C_ADDRESS_LTR390 0x53
#define OZONE_ADDRESS OZONE_ADDRESS_3  // 0x70
#define GAS_ADDRESS 0x08
#define COLLECT_NUMBER 20  // Ozone data collection range (1-100)
#define LOOP_DELAY 3000    // Main loop delay in ms
#define TIME_BUFFER_SIZE 30 // Increased buffer size for date/time
#define GPS_I2C_ADDRESS 0x42 // Default u-blox address

// Declare sensor objects
RTC_DS3231 rtc;
GAS_GMXXX<TwoWire> gas;
HP20x_dev hp20x;
DFRobot_OzoneSensor ozone;
LTR390 ltr390(I2C_ADDRESS_LTR390);
SFE_UBLOX_GNSS myGNSS; // Updated GPS object
constexpr unsigned int SENSOR_PIN {7U};
AM2302::AM2302_Sensor am2302{SENSOR_PIN};

// Function prototypes
bool initRTC();
void initGasSensor();
void initBarometer();
void initOzoneSensor();
void initUVSensor();
bool initGPS();
void updateGPS();
void printSensorData();
void printDateTime(const DateTime &dt);
void scanI2CBus(); // New function to scan I2C bus

// Global flag for sensor status
bool rtcInitialized = false;
bool gpsInitialized = false;

void setup() {
  // Initialize serial and I2C
  Serial.begin(9600); // Set to 9600 baud as requested
  Wire.begin();
  if (am2302.begin()) {
      // this delay is needed to receive valid data,
      // when the loop directly read again
   }
  
  // Wait for serial to be ready (especially on Leonardo/Micro boards)
  while (!Serial && millis() < 3000);
  
  Serial.println(F("Environmental Sensor Array Initializing..."));
  
  // Scan I2C bus to find connected devices
  scanI2CBus();
  
  // Initialize all sensors
  rtcInitialized = initRTC();
  initGasSensor();
  initBarometer();
  initOzoneSensor();
  initUVSensor();
  gpsInitialized = initGPS();
  
  // Print I2C configuration summary
  Serial.println(F("\nI2C Configuration Summary:"));
  Serial.println(F("-------------------------"));
  Serial.println(F("RTC Module: 0x68 (default)"));
  Serial.print(F("Multichannel Gas: 0x")); Serial.println(GAS_ADDRESS, HEX);
  Serial.print(F("Ozone Sensor: 0x")); Serial.println(OZONE_ADDRESS, HEX);
  Serial.print(F("LTR390 UV Sensor: 0x")); Serial.println(I2C_ADDRESS_LTR390, HEX);
  Serial.print(F("GPS: 0x42\n")); 
  if (gpsInitialized) {
    // Serial.println(myGNSS.getI2CAddress(), HEX);
  } else {
    Serial.println(F("Not connected"));
  }
  Serial.println(F("-------------------------\n"));
}

void loop() {
  // No explicit update needed for GPS - u-blox library updates automatically
  
  // Print all sensor data
  printSensorData();

  delay(LOOP_DELAY);
}

// Scan I2C bus for connected devices
void scanI2CBus() {
  Serial.println(F("Scanning I2C bus for devices:"));
  byte count = 0;
  
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print(F("Device found at address 0x"));
      if (address < 16) Serial.print(F("0"));
      Serial.println(address, HEX);
      count++;
    }
  }
  
  Serial.print(F("Found ")); 
  Serial.print(count);
  Serial.println(F(" device(s) on I2C bus\n"));
}

// Initialize Real-Time Clock - returns true if successful
bool initRTC() {
  Serial.print(F("Initializing RTC... "));
  
  // Try to initialize RTC
  if (!rtc.begin()) {
    Serial.println(F("FAILED - Check wiring"));
    return false;
  }
  
  // Uncomment the line below on first upload to set RTC to computer time
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  
  // Check if RTC lost power and warn (but don't update automatically)
  if (rtc.lostPower()) {
    Serial.println(F("OK but RTC lost power! Time may be incorrect"));
    // Don't automatically adjust time:
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    return true;
  }
  
  Serial.println(F("OK"));
  return true;
}

// Initialize Gas Sensor
void initGasSensor() {
  Serial.print(F("Initializing Gas Sensor... "));
  gas.begin(Wire, GAS_ADDRESS);
  Serial.println(F("OK"));
}

// Initialize Barometer
void initBarometer() {
  Serial.print(F("Initializing Barometer... "));
  hp20x.begin();
  Serial.println(F("OK"));
}

// Initialize Ozone Sensor
void initOzoneSensor() {
  Serial.print(F("Initializing Ozone Sensor... "));
  delay(100); // Brief delay for sensor stabilization
  
  if (!ozone.begin(OZONE_ADDRESS)) {
    Serial.println(F("FAILED - Check wiring and address"));
    Serial.println(F("Retrying..."));
    delay(1000);
    if (!ozone.begin(OZONE_ADDRESS)) {
      Serial.println(F("FAILED AGAIN - Continuing anyway"));
    } else {
      Serial.println(F("Connected on retry"));
      ozone.setModes(MEASURE_MODE_PASSIVE);
    }
  } else {
    ozone.setModes(MEASURE_MODE_PASSIVE);
    Serial.println(F("OK"));
  }
}

// Initialize UV Sensor
void initUVSensor() {
  Serial.print(F("Initializing UV Sensor... "));
  if (!ltr390.init()) {
    Serial.println(F("FAILED - Check wiring"));
  } else {
    // Configure sensor
    ltr390.setMode(LTR390_MODE_ALS);
    ltr390.setGain(LTR390_GAIN_3);
    ltr390.setResolution(LTR390_RESOLUTION_18BIT);
    Serial.println(F("OK"));
  }
}

// Initialize GPS - updated for u-blox GPS library
bool initGPS() {
  Serial.print(F("Initializing GPS... "));
  
  // First try the default address (0x42)
  if (myGNSS.begin(Wire, GPS_I2C_ADDRESS)) {
    Serial.println(F("OK at address 0x42"));
    return true;
  }
  
  // If failed, try alternative address (0x57)
  Serial.print(F("Failed at 0x42, trying 0x57... "));
  if (myGNSS.begin(Wire, 0x57)) {
    Serial.println(F("OK at address 0x57"));
    return true;
  }
  
  // If still failed, try auto-detection
  Serial.print(F("Failed at 0x57, trying auto-detection... "));
  if (myGNSS.begin(Wire)) {
    Serial.println(F("OK with auto-detection"));
    return true;
  }
  
  // All attempts failed
  Serial.println(F("FAILED - Check wiring"));
  return false;
}

// Print date and time from RTC in a reliable way
void printDateTime(const DateTime &dt) {
  // Print time
  Serial.print(F("TIME: "));
  
  // Print hours with leading zero if needed
  if (dt.hour() < 10) Serial.print('0');
  Serial.print(dt.hour());
  Serial.print(':');
  
  // Print minutes with leading zero if needed
  if (dt.minute() < 10) Serial.print('0');
  Serial.print(dt.minute());
  Serial.print(':');
  
  // Print seconds with leading zero if needed
  if (dt.second() < 10) Serial.print('0');
  Serial.print(dt.second());
  
  // Print date
  Serial.print(F(" | DATE: "));
  
  // Print month with leading zero if needed
  if (dt.month() < 10) Serial.print('0');
  Serial.print(dt.month());
  Serial.print('/');
  
  // Print day with leading zero if needed
  if (dt.day() < 10) Serial.print('0');
  Serial.print(dt.day());
  Serial.print('/');
  
  // Print year
  Serial.println(dt.year());
}

// Print all sensor data
void printSensorData() {
  Serial.println(F("\n=========== SENSOR READINGS ==========="));
  
  // SECTION 1: Time and Date (with error handling)
  if (rtcInitialized) {
    DateTime now = rtc.now();
    printDateTime(now);
  } else {
    Serial.println(F("RTC not initialized - No time data"));
  }
  
  // SECTION 2: GPS Data - updated for u-blox GPS
  Serial.println(F("\n-- GPS Data --"));
  if (gpsInitialized) {
    if (myGNSS.isConnected()) {
      // Get positioning data using u-blox methods
      long latitude = myGNSS.getLatitude();
      long longitude = myGNSS.getLongitude();
      long altitude = myGNSS.getAltitude();
      byte satellites = myGNSS.getSIV();
      byte fixType = myGNSS.getFixType();
      
      // Convert and print location
      Serial.print(F("Location: "));
      Serial.print(latitude / 10000000.0, 6); // Convert to decimal degrees
      Serial.print(F(", "));
      Serial.println(longitude / 10000000.0, 6);
      
      // Print altitude
      Serial.print(F("Altitude: "));
      Serial.print(altitude / 1000.0); // Convert to meters
      Serial.println(F(" m"));
      
      // Print satellites
      Serial.print(F("Satellites: "));
      Serial.println(satellites);
      
      // Print fix type
      Serial.print(F("Fix type: "));
      if (fixType == 0) Serial.println(F("No fix"));
      else if (fixType == 1) Serial.println(F("Dead reckoning"));
      else if (fixType == 2) Serial.println(F("2D fix"));
      else if (fixType == 3) Serial.println(F("3D fix"));
      else if (fixType == 4) Serial.println(F("GNSS + Dead reckoning"));
      else if (fixType == 5) Serial.println(F("Time only"));
    } else {
      Serial.println(F("GPS not connected"));
    }
  } else {
    Serial.println(F("GPS not initialized"));
  }
  
  // SECTION 3: Gas Sensor Data
  Serial.println(F("\n-- Gas Sensor --"));
  Serial.print(F("NO2: ")); Serial.print(gas.measure_NO2()); Serial.println(F(" ppm"));
  Serial.print(F("C2H5OH: ")); Serial.print(gas.measure_C2H5OH()); Serial.println(F(" ppm"));
  Serial.print(F("VOC: ")); Serial.print(gas.measure_VOC()); Serial.println(F(" ppm"));
  Serial.print(F("CO: ")); Serial.print(gas.measure_CO()); Serial.println(F(" ppm"));
  
  // SECTION 4: Barometer Data
  Serial.println(F("\n-- Barometer --"));
  float pressure = hp20x.ReadPressure() / 100.0;
  float altitude = hp20x.ReadAltitude() / 100.0;
  Serial.print(F("Pressure: ")); Serial.print(pressure); Serial.println(F(" hPa"));
  Serial.print(F("Altitude: ")); Serial.print(altitude); Serial.println(F(" meters"));
  
  // SECTION 5: Ozone Data
  Serial.println(F("\n-- Ozone Sensor --"));
  int16_t ozoneConcentration = ozone.readOzoneData(COLLECT_NUMBER);
  Serial.print(F("Ozone: ")); Serial.print(ozoneConcentration); Serial.println(F(" PPB"));
  
  // SECTION 6: UV/Light Data
  Serial.println(F("\n-- UV/Light Sensor --"));
  if (ltr390.newDataAvailable()) {
    if (ltr390.getMode() == LTR390_MODE_ALS) {
      Serial.print(F("Ambient Light: ")); 
      Serial.print(ltr390.getLux());
      Serial.println(F(" Lux"));
      
      // Swap to UV mode for next reading
      ltr390.setGain(LTR390_GAIN_18);
      ltr390.setResolution(LTR390_RESOLUTION_20BIT);
      ltr390.setMode(LTR390_MODE_UVS);
    } else {
      Serial.print(F("UV Index: ")); 
      Serial.println(ltr390.getUVI());
      
      // Swap to ALS mode for next reading
      ltr390.setGain(LTR390_GAIN_3);
      ltr390.setResolution(LTR390_RESOLUTION_18BIT);
      ltr390.setMode(LTR390_MODE_ALS);
    }
  } else {
    Serial.println(F("No new UV/Light data available"));
  }

  // Section 7: Temperature and Humidity
  auto status = am2302.read();
   Serial.print("\n\nstatus of sensor read(): ");
   Serial.println(status);

   Serial.print("Temperature: ");
   Serial.println(am2302.get_Temperature());

   Serial.print("Humidity:    ");
   Serial.println(am2302.get_Humidity());
  
  Serial.println(F("======================================="));
}
