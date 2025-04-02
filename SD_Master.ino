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
#define CAMERA_DELAY 120000
#define TIME_BUFFER_SIZE 30 // Increased buffer size for date/time
#define GPS_I2C_ADDRESS 0x42 // Default u-blox address
#define FRAMES_NUM 0x00 //Amount of photos taken

//Camera + SD Card constants


#define SD_CS 6
const int Cam_CS = 10;
ArduCAM myCAM (OV2640, Cam_CS);
bool is_header = false;

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
bool CameraHealthCheck();
uint8_t read_fifo_burst(ArduCAM myCAM);
void saveSensorData();
void CameraToSD();

// Global flag for sensor status
bool rtcInitialized = false;
bool gpsInitialized = false;

DateTime previous_time;

void setup() {
  // Initialize serial and I2C
  Serial.begin(115200); // Set to 115200 baud as requested
  Wire.begin();
  pinMode(Cam_CS, OUTPUT);
  digitalWrite(Cam_CS, HIGH);
  SPI.begin();
  SD.begin(SD_CS);
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
    Serial.println(F("GPS not connected"));
  }
  Serial.println(F("-------------------------\n"));

  if(rtc.begin())
  {
    previous_time = rtc.now();
  }
}

void loop() {
  DateTime current_time = rtc.now();
  //Save all sensor data
  if (SD.begin(SD_CS))
  {
    saveSensorData();
  }

  if((current_time.unixtime()-previous_time.unixtime() >= CAMERA_DELAY)&&SD.begin(SD_CS)&&CameraHealthCheck()){
    CameraToSD();
  }

  


  // No explicit update needed for GPS - u-blox library updates automatically
  
  // Print all sensor data
  //printSensorData();

  delay(LOOP_DELAY);
}

bool CameraHealthCheck(){
  uint8_t vid, pid;
  uint8_t temp;
  //Test SPI Interface
  myCAM.write_reg(0x07, 0x80);
  delay(100);
  myCAM.write_reg(0x07, 0x00);
  delay(100);

  //Check if the ArduCAM SPI bus is OK
  myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
  temp = myCAM.read_reg(ARDUCHIP_TEST1);
  if (temp != 0x55){

    Serial.println(F("SPI interface Error!"));
    delay(1000);
    return false;
  }
  else {
    Serial.println(F("SPI interface OK."));

    //Ensure correcct camera module

    myCAM.wrSensorReg8_8(0xff, 0x01);
    myCAM.rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
    myCAM.rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);
    if ((vid != 0x26 ) && (( pid != 0x41 ) || ( pid != 0x42 ))) {
      Serial.println(F("ACK CMD Can't find OV2640 module!"));
      delay(1000);
      return false;
    }
    else{
      Serial.println(F("ACK CMD OV2640 detected.")); 
      myCAM.InitCAM();
      myCAM.flush_fifo();
      myCAM.clear_fifo_flag();
      myCAM.write_reg(ARDUCHIP_FRAMES, FRAMES_NUM);
      return true;
    }
  }
}

uint8_t read_fifo_burst(ArduCAM myCAM)
{
  uint8_t temp = 0, temp_last = 0;
  uint32_t length = 0;
  static int i = 0;
  static int k = 0;
  char str[16];
  File outFile;
  byte buf[256];
  length = myCAM.read_fifo_length();
  Serial.print(F("The fifo length is :"));
  Serial.println(length, DEC);
  if (length >= MAX_FIFO_SIZE) //8M
  {
    Serial.println("Over size.");
    return 0;
  }
  if (length == 0 ) //0 kb
  {
    Serial.println(F("Size is 0."));
    return 0;
  }
  myCAM.CS_LOW();
  myCAM.set_fifo_burst();//Set fifo burst mode
  i = 0;
  while ( length-- )
  {
    temp_last = temp;
    temp =  SPI.transfer(0x00);
    //Read JPEG data from FIFO
    if ( (temp == 0xD9) && (temp_last == 0xFF) ) //If find the end ,break while,
    {
      buf[i++] = temp;  //save the last  0XD9
      //Write the remain bytes in the buffer
      myCAM.CS_HIGH();
      outFile.write(buf, i);
      //Close the file
      outFile.close();
      Serial.println(F("OK"));
      is_header = false;
      myCAM.CS_LOW();
      myCAM.set_fifo_burst();
      i = 0;
    }
    if (is_header == true)
    {
      //Write image data to buffer if not full
      if (i < 256)
        buf[i++] = temp;
      else
      {
        //Write 256 bytes image data to file
        myCAM.CS_HIGH();
        outFile.write(buf, 256);
        i = 0;
        buf[i++] = temp;
        myCAM.CS_LOW();
        myCAM.set_fifo_burst();
      }
    }
    else if ((temp == 0xD8) & (temp_last == 0xFF))
    {
      is_header = true;
      myCAM.CS_HIGH();
      //Create a avi file
      k = k + 1;
      itoa(k, str, 10);
      strcat(str, ".jpg");
      //Open the new file
      outFile = SD.open(str, O_WRITE | O_CREAT | O_TRUNC);
      if (! outFile)
      {
        Serial.println(F("File open failed"));
        while (1);
      }
      myCAM.CS_LOW();
      myCAM.set_fifo_burst();
      buf[i++] = temp_last;
      buf[i++] = temp;
    }
  }
  myCAM.CS_HIGH();
  return 1;
}

void CameraToSD(){
  //Clear camera register
  myCAM.flush_fifo();
  myCAM.clear_fifo_flag();
  myCAM.OV2640_set_JPEG_size(OV2640_1600x1200);
  myCAM.start_capture();
  //Capture until ending bit is read
  while ( !myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK));
  read_fifo_burst(myCAM);
  myCAM.clear_fifo_flag();
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
  
  Serial.println(F("======================================="));
  auto status = am2302.read();
   Serial.print("\n\nstatus of sensor read(): ");
   Serial.println(status);

   Serial.print("Temperature: ");
   Serial.println(am2302.get_Temperature());

   Serial.print("Humidity:    ");
   Serial.println(am2302.get_Humidity());
  
  Serial.println(F("======================================="));
}


void saveSensorData() {
  static int p = 0;
  char str2[16];
  File dataFile;
  p = p + 1;
  itoa(p, str2, 10);
  strcat(str2, ".txt");
  dataFile = SD.open(str2, O_WRITE | O_CREAT | O_TRUNC );
  if(!dataFile)
  {
    return;
  }
  dataFile.println(F("\n=========== SENSOR READINGS ==========="));
  
  // SECTION 1: Time and Date (with error handling)
  if (rtcInitialized) {
    DateTime now = rtc.now();
    printDateTime(now);
  } else {
    dataFile.println(F("RTC not initialized - No time data"));
  }
  
  // SECTION 2: GPS Data - updated for u-blox GPS
  dataFile.println(F("\n-- GPS Data --"));
  if (gpsInitialized) {
    if (myGNSS.isConnected()) {
      // Get positioning data using u-blox methods
      long latitude = myGNSS.getLatitude();
      long longitude = myGNSS.getLongitude();
      long altitude = myGNSS.getAltitude();
      byte satellites = myGNSS.getSIV();
      byte fixType = myGNSS.getFixType();
      
      // Convert and print location
      dataFile.print(F("Location: "));
      dataFile.print(latitude / 10000000.0, 6); // Convert to decimal degrees
      dataFile.print(F(", "));
      dataFile.println(longitude / 10000000.0, 6);
      
      // Print altitude
      dataFile.print(F("Altitude: "));
      dataFile.print(altitude / 1000.0); // Convert to meters
      dataFile.println(F(" m"));
      
      // Print satellites
      dataFile.print(F("Satellites: "));
      dataFile.println(satellites);
      
      // Print fix type
      dataFile.print(F("Fix type: "));
      if (fixType == 0) dataFile.println(F("No fix"));
      else if (fixType == 1) dataFile.println(F("Dead reckoning"));
      else if (fixType == 2) dataFile.println(F("2D fix"));
      else if (fixType == 3) dataFile.println(F("3D fix"));
      else if (fixType == 4) dataFile.println(F("GNSS + Dead reckoning"));
      else if (fixType == 5) dataFile.println(F("Time only"));
    } else {
      dataFile.println(F("GPS not connected"));
    }
  } else {
    dataFile.println(F("GPS not initialized"));
  }
  
  // SECTION 3: Gas Sensor Data
  dataFile.println(F("\n-- Gas Sensor --"));
  dataFile.print(F("NO2: ")); dataFile.print(gas.measure_NO2()); dataFile.println(F(" ppm"));
  dataFile.print(F("C2H5OH: ")); dataFile.print(gas.measure_C2H5OH()); dataFile.println(F(" ppm"));
  dataFile.print(F("VOC: ")); dataFile.print(gas.measure_VOC()); dataFile.println(F(" ppm"));
  dataFile.print(F("CO: ")); dataFile.print(gas.measure_CO()); dataFile.println(F(" ppm"));
  
  // SECTION 4: Barometer Data
  dataFile.println(F("\n-- Barometer --"));
  float pressure = hp20x.ReadPressure() / 100.0;
  float altitude = hp20x.ReadAltitude() / 100.0;
  dataFile.print(F("Pressure: ")); dataFile.print(pressure); dataFile.println(F(" hPa"));
  dataFile.print(F("Altitude: ")); dataFile.print(altitude); dataFile.println(F(" meters"));
  
  // SECTION 5: Ozone Data
  dataFile.println(F("\n-- Ozone Sensor --"));
  int16_t ozoneConcentration = ozone.readOzoneData(COLLECT_NUMBER);
  dataFile.print(F("Ozone: ")); dataFile.print(ozoneConcentration); dataFile.println(F(" PPB"));
  
  // SECTION 6: UV/Light Data
  dataFile.println(F("\n-- UV/Light Sensor --"));
  if (ltr390.newDataAvailable()) {
    if (ltr390.getMode() == LTR390_MODE_ALS) {
      dataFile.print(F("Ambient Light: ")); 
      dataFile.print(ltr390.getLux());
      dataFile.println(F(" Lux"));
      
      // Swap to UV mode for next reading
      ltr390.setGain(LTR390_GAIN_18);
      ltr390.setResolution(LTR390_RESOLUTION_20BIT);
      ltr390.setMode(LTR390_MODE_UVS);
    } else {
      dataFile.print(F("UV Index: ")); 
      dataFile.println(ltr390.getUVI());
      
      // Swap to ALS mode for next reading
      ltr390.setGain(LTR390_GAIN_3);
      ltr390.setResolution(LTR390_RESOLUTION_18BIT);
      ltr390.setMode(LTR390_MODE_ALS);
    }
  } else {
    dataFile.println(F("No new UV/Light data available"));
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
  dataFile.close();
}
