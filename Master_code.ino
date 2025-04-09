/* 
 * Main Mission Code, satelite branch
 * For Wiring Diagram and Extra Notes, See Wiring_and_Notes.txt
 * User Defined Variables:
  - MAIN_BAUD 
  - MAIN_LOOP_DELAY
*/

// Libraries
#include <IridiumSBD.h>                           // https://github.com/mikalhart/IridiumSBD
#include <Wire.h>                                 // Included with Arduino
#include <RTClib.h>                               // https://github.com/adafruit/RTClib
#include <Multichannel_Gas_GMXXX.h>               // https://github.com/Seeed-Studio/Seeed_Arduino_MultiGas
#include <HP20x_dev.h>                            // https://github.com/Seeed-Studio/Grove_Barometer_HP20x
#include "DFRobot_OzoneSensor.h"                  // https://github.com/DFRobot/DFRobot_OzoneSensor
#include <LTR390.h>                               // https://github.com/levkovigor/LTR390
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> // https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library
#include <AM2302-Sensor.h>

// I2C Intrerface Addresses
#define I2C_ADDRESS_LTR390 0x53
#define OZONE_ADDRESS OZONE_ADDRESS_3  // 0x70
#define GAS_ADDRESS 0x08
#define TIME_BUFFER_SIZE 30 // Increased buffer size for date/time
#define GPS_I2C_ADDRESS 0x42 // Default u-blox address
#define IridiumSerial Serial3 // serial for satelite commmunication
#define MAX_PACKETS 4

// Declare Sensor objects
RTC_DS3231 rtc;
SFE_UBLOX_GNSS myGNSS;
GAS_GMXXX<TwoWire> gas;
HP20x_dev hp20x;
DFRobot_OzoneSensor ozone;
LTR390 ltr390(I2C_ADDRESS_LTR390);
constexpr unsigned int SENSOR_PIN {7U}; // Pin 7 for Temperaute/Humidity
AM2302::AM2302_Sensor am2302{SENSOR_PIN};
IridiumSBD IridiumModem(IridiumSerial);

/* 
  55 Byte Single Sensor Packet Structure (equal to one line of data)
  The __attribyte__((packed)) command makes sure it is exactly 55 by eliminating padding by the compiler.
  Furthermore, there are 3 bytes included in the final message (1 for complete packet count, and 2 for xor checksum)
  Sizes: 
  - uint8_t  : 8  Bits = 1 Byte
  - uint16_t : 16 Bits = 2 Bytes
  - uint32_t : 32 Bits = 4 Bytes
  - float    : 32 Bits = 4 Bytes
*/ 
struct __attribute__((packed)) DataPacket {
  // Time (3 Bytes)
  uint8_t second;
  uint8_t minute;
  uint8_t hour;

  // GPS (14 Bytes)
  uint32_t latitude;
  uint32_t longitude;
  uint32_t altitude;
  uint8_t satellites;
  uint8_t fixtype;

  // Sensors (38 Bytes)
  uint32_t NO2;
  uint32_t C2H5OH;
  uint32_t VOC;
  uint32_t CO;
  uint16_t ozone;
  float pressure;
  float uv;
  float lux;
  float temperature;
  float humidity;
};

// Time reference holder
struct TimeRef {
  DateTime rtcTime;
  unsigned long sysMillis;
};

bool rtcInitialized = false;
bool gpsInitialized = false;
// Global variables
DataPacket packetBuffer[MAX_PACKETS];
uint8_t packetCount = 0;

// Function Prototypes
bool initRTC();
bool initGPS();
bool initGasSensor();
bool initBarometer();
bool initOzoneSensor();
bool initUVSensor();
bool sendSBDMessage();
DataPacket collectData();
bool hasSecondsPassed(TimeRef& last, uint32_t seconds); // last needs to have both values for rtc and millis.

// Constants
#define MAIN_LOOP_DELAY 3000
#define PACKET_SIZE sizeof(DataPacket)
#define COLLECT_NUMBER 20  // Ozone data collection range (1-100)
#define MAIN_BAUD 9600
#define MESSAGE_INTERVAL 180 // seconds

void setup() {
  // Initialize serial and I2C
  Serial.begin(MAIN_BAUD); // Set Serial for Printing to 9600 baud as requested
  Wire.begin();

  TimeRef last_message;
  TimeRef last_packet; // being averaged
  TimeRef last_;
  
  // Initialize all sensors
  bool rtcInitialized = initRTC();
  bool gpsInitialized = initGPS();
  initGasSensor();
  initBarometer();
  initOzoneSensor();
  initUVSensor();
  initIridium();

  last_message.rtcTime = rtc.now();
  last_packet.rtcTime = rtc.now(); // being averaged
  last_message.sysMillis = millis();
  last_packet.sysMillis = millis();
  last_.rtcTime = rtc.now(); // being averaged
  last_.sysMillis = millis();
}

void loop() {
  // Collect sensor data every X seconds into packer form collectData()
  // write them to sd card
  // add them for averaging, keep track of count
  // add LED light here

  // every 1 minute, save averaged packet to packetBUffer according to it's count, up to 4
  // every 3 minutes, when packet buffer is ready, start new satellite transmission.

  // UPDATE SBD_CALLBACK to include writing, collecting data every 5 seconds, and  continuously led blinking.
  // ADD MAX VALUES TO SENSOR IF FAIL


    // Send SBD message every 60 seconds if we have data
  delay(MAIN_LOOP_DELAY);
}

bool initIridium() {
  IridiumSerial.begin(19200);  // Iridium SBD baud rate
  IridiumModem.begin();        // Initialize the Iridium IridiumModem
  return true;
}
// Initialize Real-Time Clock - returns true if successful
bool initRTC() {
  // Try to initialize RTC
  if (!rtc.begin()) {
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
  return true;
}

// Initialize Gas Sensor FIXME: Add false conditional (gas.begin has void return)
bool initGasSensor() {
  gas.begin(Wire, GAS_ADDRESS);
}

// Initialize Barometer FIXME: add false conditional
bool initBarometer() { 
  hp20x.begin();
}

// Initialise Ozone sensor
bool initOzoneSensor() {
  delay(100); // Brief delay for sensor stabilization
  
  if (!ozone.begin(OZONE_ADDRESS)) {
    return false;
  } 
  else {
    ozone.setModes(MEASURE_MODE_PASSIVE);
  }
}

// Initialize UV Sensor
bool initUVSensor() {
  if (!ltr390.init()) return false;
  else {
    // Configure sensor
    ltr390.setMode(LTR390_MODE_ALS);
    ltr390.setGain(LTR390_GAIN_3);
    ltr390.setResolution(LTR390_RESOLUTION_18BIT);
    Serial.println(F("OK"));
  }
  return true;

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
  if (myGNSS.begin(Wire, 0x57)) {
    return true;
  }
  
  // If still failed, try auto-detection
  if (myGNSS.begin(Wire)) {
    return true;
  }
  
  // All attempts failed
  return false;
}

// Initialize Temperature 
bool initTemperatureSensor() {
    am2302.begin();  // Initialize the AM2302 sensor

    delay(1000);  // Small delay to allow stabilization

    if (am2302.read() != 0) {  // Check if the sensor responds correctly
        return true;
    } else {
        return false;
    }
}

// Returns Time info by reference to packet.
bool getDateTime(DataPacket &packet) {
  if (!rtcInitialized) {
    return false;
  } 
  DateTime now = rtc.now();
  packet.hour = now.hour();
  packet.minute = now.minute();
  packet.second = now.second();
  // packet.month = now.month();
  // packet.day = now.day();
  // packeet.year = now.year();
  return true;
}

bool hasSecondsPassed(TimeRef& last, uint32_t seconds) {
  if (rtcInitialized) {
    DateTime now = rtc.now();
    TimeSpan elapsed = now - last.rtcTime;
    if (elapsed.totalseconds() >= seconds) {
      last.rtcTime = now;
      last.sysMillis = millis(); // Keep both updated
      return true;
    }
    return false;
  } else {
    unsigned long now = millis();
    if ((now - last.sysMillis) >= seconds * 1000UL) {
      last.sysMillis = now;
      return true;
    }
    return false;
  }
}
// Get GPS function to fill the DataPacket struct by reference
bool getGPS(DataPacket &packet) {
  if (gpsInitialized) {
    if (myGNSS.isConnected()) {
      // Get positioning data using u-blox methods
      packet.latitude = myGNSS.getLatitude();  // Save latitude to packet
      packet.longitude = myGNSS.getLongitude();  // Save longitude to packet
      packet.altitude = myGNSS.getAltitude();  // Save altitude to packet
      packet.satellites = myGNSS.getSIV();  // Save satellites count to packet
      packet.fixtype = myGNSS.getFixType();  // Save GPS fix type to packet
      return true;

      // Serial.print(latitude / 10000000.0, 6); // Convert to decimal degrees (usually they are scaled by 1e7)
      // Serial.println(longitude / 10000000.0, 6); // convert to meters from millimeters
      // FIXME: Check is meters or km or what

      // if (fixType == 0) Serial.println(F("No fix"));
      // else if (fixType == 1) Serial.println(F("Dead reckoning"));
      // else if (fixType == 2) Serial.println(F("2D fix"));
      // else if (fixType == 3) Serial.println(F("3D fix"));
      // else if (fixType == 4) Serial.println(F("GNSS + Dead reckoning"));
      // else if (fixType == 5) Serial.println(F("Time only"));
    } 
    else {
      return false;
    }
  } 
  else {
    return false;
  }
}

// FIXME add false conditional to function Get Gas Sensor Data
bool getGas(DataPacket &packet) {
    packet.NO2 = gas.measure_NO2();       // NO2 concentration in ppm
    packet.C2H5OH = gas.measure_C2H5OH(); // Ethanol concentration in ppm
    packet.VOC = gas.measure_VOC();       // VOC concentration in ppm
    packet.CO = gas.measure_CO();         // CO concentration in ppm
    return true;
}

// 
bool getBaro(DataPacket &packet) {
  float pressure = hp20x.ReadPressure() / 100.0;  // Convert to hPa
  float altitude = hp20x.ReadAltitude() / 100.0;  // Convert to meters
  if (pressure > 0) {  // Assuming a valid reading is nonzero
    packet.pressure = pressure;
    packet.altitude = altitude;
    return true;
  } else {
    return false;
  }
}

// Get UV Sensor Data. (Measures Ambient Light and UVI)
bool getUV(DataPacket &packet) {
    if (!ltr390.newDataAvailable()) {
        return false;
    }
    // Read Ambient Light (Lux)
    ltr390.setGain(LTR390_GAIN_3);
    ltr390.setResolution(LTR390_RESOLUTION_18BIT);
    ltr390.setMode(LTR390_MODE_ALS);
    delay(100);  // Small delay to allow sensor to switch modes

    packet.lux = ltr390.getLux();  // Save Lux to packet

    // Read UV Index
    ltr390.setGain(LTR390_GAIN_18);
    ltr390.setResolution(LTR390_RESOLUTION_20BIT);
    ltr390.setMode(LTR390_MODE_UVS);
    delay(100);  // Small delay to allow sensor to switch modes

    packet.uv = ltr390.getUVI();  // Save UVI to packet
    return true;
}

// Get Ozone Sensor Data
bool getOzone(DataPacket &packet) {
  int16_t ozoneConcentration = ozone.readOzoneData(COLLECT_NUMBER);
  if (ozoneConcentration <= 0) return false;

    // Valid ozone data
    packet.ozone = ozoneConcentration;
    return true;
}

// Get Temperature and Humidity Data
bool getTempHumidity(DataPacket &packet) {
  auto status = am2302.read();
  if (status == 0) {  // Assuming 0 means a successful read
    packet.temperature = am2302.get_Temperature();
    packet.humidity= am2302.get_Humidity();
  } else {
    return false;
  }
  return true;
}

// Function to send SBD message using Iridium SBD
bool sendSBDMessage() {
    if (packetCount == 0) return;  // No data to send
    int messageSize = 1 + (packetCount * sizeof(DataPacket));

    // Prepare the message (1 byte for packet count, followed by packet data)
    uint8_t message[messageSize];  // Total message size (array of bytes)
    message[0] = packetCount;  // First byte is the packet count
    memcpy(&message[1], packetBuffer, messageSize - 1);  // Copy packets into message

    // Send the message using the IridiumSBD library's sendSBDBinary function
    IridiumModem.sendSBDBinary(message, messageSize);

    // Reset packet buffer
    packetCount = 0;
    return true;
}

// FIXME: Test function to collect data and return a packet
DataPacket collectData() {
 DataPacket packet;
    getDateTime(packet);
    getGPS(packet);
    getUV(packet);
    getBaro(packet);
    getGas(packet);
    getOzone(packet);
    getTempHumidity(packet);
    return packet;
}
