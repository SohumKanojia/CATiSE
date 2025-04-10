/*
Project Calico - CubeCats CATiSE Program Fall 2024-Spring 2025
 * Main Mission Code
 * Contributers:
  - Name, Role or function and so on
  -

 * For Wiring Diagram and Extra Notes, See Wiring_and_Notes.txt
 * User Defined Variables:
    All Parameters
    Iridium Serial: Serial3
    Temperaute Sensor Digital Pin: 7

*/

// Mission Parameters (Time in seconds)
  #define MAIN_BAUD 9600          // Baud rate used for Serial Monitor Printing
  #define SENSOR_INTERVAL 1       // Minimum interval between sensor collection
  #define PACKET_INTERVAL 45      // Minimum Span to average a packet over
  #define MESSAGE_INTERVAL 180    // Minimum interval between Sending Messages to Satellite
  #define PICTURE_INTERVAL 300    // Minimum interval between taking a picture for local storage 
  #define PACKETS_PER_MESSAGE 4   // Number of Packets in every message to be sent to the sattelite (proportional to max size of message 340, and packet size)
  #define MESSAGE_SIZE 220        // size of each packet in the message * the count of them
  #define LED_PIN 5
  #define TEMPHUMID_PIN 7

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
  #define TIME_message_SIZE 30 // Increased message size for date/time
  #define GPS_I2C_ADDRESS 0x42 // Default u-blox address
  #define IridiumSerial Serial3 // serial for satelite commmunication



// Sensor objects & Some ports
  RTC_DS3231 rtc;
  SFE_UBLOX_GNSS myGNSS;
  GAS_GMXXX<TwoWire> gas;
  HP20x_dev hp20x;
  DFRobot_OzoneSensor ozone;
  LTR390 ltr390(I2C_ADDRESS_LTR390);
  constexpr unsigned int SENSOR_PIN {7U}; // Pin 7 for Temperaute/Humidity
  AM2302::AM2302_Sensor am2302{SENSOR_PIN};
            // Digital pin controlling LEDs
  IridiumSBD IridiumModem(IridiumSerial);
  #define COLLECT_NUMBER 20       // Ozone data collection range (1-100)
  
// DataPacket Struct Info:
  struct __attribute__((packed)) DataPacket {
    /* Info
      55 Byte Sensor Packet Structure (equal to one line of data)
      The __attribyte__((packed)) command makes sure it is exactly 55 by eliminating padding by the compiler.
      Furthermore, there are 3 bytes included in the final message (1 for complete packet count, and 2 for xor checksum)
      Sizes: 
      - uint8_t  : 8  Bits = 1 Byte
      - uint16_t : 16 Bits = 2 Bytes
      - uint32_t : 32 Bits = 4 Bytes
      - float    : 32 Bits = 4 Bytes
    */ 

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

// Time Reference Structure
  struct TimeRef {
    // Used to store rtc time object and millis object. Works with hasSecondspassed() function
    DateTime rtcTime;
    unsigned long sysMillis;
  };

// Function Prototypes
  // (The getSensor group of functions are not here since they are called by collectData())
  bool initRTC();
  bool initGPS();
  bool initGasSensor();
  bool initOzoneSensor();
  bool initBarometer();
  bool initUVSensor();
  bool initIridium();
  bool hasSecondsPassed(TimeRef& last, uint32_t seconds); // last needs to have both values for rtc and millis.
  DataPacket collectData();
  bool sendSBDMessage();
  void setLights(bool on, int startDelay = 0, int endDelay = 0);



// Loop Variables
  bool rtcInitialized = false;
  bool gpsInitialized = false;
  DataPacket buffer[PACKETS_PER_MESSAGE]; // message with Message Size
  uint8_t message[MESSAGE_SIZE];
  uint32_t readingCount = 0;
  uint8_t messagePacketCount = 0; // stores the current
  TimeRef last_m; // time of last message
  TimeRef last_p; // time of last averaged packet
  TimeRef last_;  // time of last update of last_ (used to track sensors)

void setup() {
  // Initialize serial and I2C
  Serial.begin(MAIN_BAUD); // Set Serial for Printing to 9600 baud as requested
  Wire.begin();
  
  // Initialize all parts
  bool rtcInitialized = initRTC();
  bool gpsInitialized = initGPS();
  initGasSensor();
  initOzoneSensor();
  initBarometer();
  initUVSensor();
  initIridium();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);   // Start with LEDs ON

  // Setup Initial times.
  last_m.rtcTime = rtc.now();
  last_m.sysMillis = millis();
  last_p = last_m;
  last_ = last_m;
}

void loop() {
  // OFF/ON for LED's

  /* ============ COLLECTION PART (envelop in collectFunc()) Trent is working on this, if necessary to be a function, would have to combine with packeting part to store totals together.
    or global initialisation of totals.
    TAKE PICTURE FUNCTION every PICTURE_INTERVAL
    Collect sensor data every SENSOR_INTERVAL into packet form collectData()
    OFF/ON for LED's    
    write to sd card with saveSD function (pass the packet).
    Update totals for averaging. (maybe write a function with static variables?)
    readingcount ++
    returns last packet
  */

  DataPacket p; 
  p = collectData();
  // =========== PACKETING PART FIXME: (NEEDS SMALL FIX to work with COLLECTION PART): Averages Packets for Transmission Part.
  if (hasSecondsPassed(last_p, PACKET_INTERVAL)) {
    // FIXME: The total are not initiliased in the scope. Ask Trent how he wants to do it.
    // Only Average Sensory Values, GPS and TIME left as last reading
    // p.NO2         = NO2_total         / readingCount;
    // p.C2H5OH      = C2H5OH_total      / readingCount;
    // p.NO2         = NO2_total         / readingCount;
    // p.C2H5OH      = C2H5OH_total      / readingCount;
    // p.VOC         = VOC_total         / readingCount;
    // p.ozone       = ozone_total       / readingCount;
    // p.pressure    = pressure_total    / readingCount;
    // p.uv          = uv_total          / readingCount;
    // p.lux         = lux_total         / readingCount;
    // p.temperature = temperature_total / readingCount;
    // p.humidity    = humidity_total    / readingCount;

    buffer[messagePacketCount] = p;
    messagePacketCount = (messagePacketCount + 1) % 5; // keep it 4 or less, which is necessary for indexing

    // ON OFF LED
  }

  // =========== TRANSMISSION PART: Initiates (Message Array of Packets) Tranmission
  if ((messagePacketCount == 4) && hasSecondsPassed(last_m, MESSAGE_INTERVAL)) {
    messagePacketCount = 0;
    sendSBDMessage();
    // ON OFF LED
  }
    
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

// Initialize Gas Sensor FIXME: Add false conditional (gas.begin has void return)
bool initGasSensor() {
  gas.begin(Wire, GAS_ADDRESS);
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

// Initialize Barometer FIXME: add false conditional
bool initBarometer() { 
  hp20x.begin();
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

bool initIridium() {
  IridiumSerial.begin(19200);  // Iridium SBD baud rate
  IridiumModem.begin();        // Initialize the Iridium IridiumModem
  return true;
}

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

bool getGas(DataPacket &packet) {
    packet.NO2 = gas.measure_NO2();       // NO2 concentration in ppm
    packet.C2H5OH = gas.measure_C2H5OH(); // Ethanol concentration in ppm
    packet.VOC = gas.measure_VOC();       // VOC concentration in ppm
    packet.CO = gas.measure_CO();         // CO concentration in ppm
    return true;
}

bool getOzone(DataPacket &packet) {
  int16_t ozoneConcentration = ozone.readOzoneData(COLLECT_NUMBER);
  if (ozoneConcentration <= 0) return false;

    // Valid ozone data
    packet.ozone = ozoneConcentration;
    return true;
}

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

// Function to send SBD message using Iridium SBD
bool sendSBDMessage() {
    // Copy current buffer contents into message (to not conflict with the buffer being updated by the loop)
    memcpy(message, buffer, MESSAGE_SIZE);

    // Send the message
    IridiumModem.sendSBDBinary(message, MESSAGE_SIZE);
    return true;
}

// Background Function that is ran behind the scenes while the rockblock is sending a message. Can terminate whole data sendin by returning false.
// For more info see: https://github.com/mikalhart/IridiumSBD/blob/master/extras/IridiumSBD%20Arduino%20Library%20Documentation.pdf
bool ISBDCallback() {
    DataPacket p;
    // COLLECTION PART (SAME AS MAIN LOOP, Except with longer interval for sensing, depends on testing)
    // if (hasSecondsPassed(last_, 5 * SENSOR_INTERVAL)) {
      p = collectData();

    // PACKETIN PART (SAME AS MAIN LOOP , EXCEPT has message count to break loop if took too long except with longer intervals for packeting, depends on testing)

    if (hasSecondsPassed(last_p, PACKET_INTERVAL)) {
    // FIXME: The total are not initiliased in the scope. Ask Trent how he wants to do it.
    // Only Average Sensory Values, GPS and TIME left as last reading
    // p.NO2         = NO2_total         / readingCount;
    // p.C2H5OH      = C2H5OH_total      / readingCount;
    // p.NO2         = NO2_total         / readingCount;
    // p.C2H5OH      = C2H5OH_total      / readingCount;
    // p.VOC         = VOC_total         / readingCount;
    // p.ozone       = ozone_total       / readingCount;
    // p.pressure    = pressure_total    / readingCount;
    // p.uv          = uv_total          / readingCount;
    // p.lux         = lux_total         / readingCount;
    // p.temperature = temperature_total / readingCount;
    // p.humidity    = humidity_total    / readingCount;

    buffer[messagePacketCount] = p;
    messagePacketCount++;
    if (messagePacketCount == 4) {
    return false; // Breaks current transmission
  }
  
  }

  // CANNOT RUN sendSBDMessage from here. However, can return false if took very long time by adjusting in IridiumSBD:: if new message is ready to terminate current message sending, or adjust timeout.
}

// Turns LED on/off with optional delays. Assumes active means low LED voltage. Delay taken in millis.
void setLights(bool on, int startDelay = 0, int endDelay = 0) {
  delay(startDelay);
  digitalWrite(LED_PIN, (on ? LOW : HIGH));  // LOW = ON (active-low)
  delay(endDelay);
}
