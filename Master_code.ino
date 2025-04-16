/*
Project Calico - CubeCats CATiSE Program Fall 2024-Spring 2025
 * Main Mission Code
 * For Wiring Diagram and Extra Notes, See Wiring_and_Notes.txt
* User Digital Inputs to Arudino:
    Iridium Serial: Serial3
    Temperaute Sensor Digital Pin: 7
    Camera Digital Pin: 6
    Led Digital Pin: 5


*/

// Mission Parameters (Time in seconds)
  #define MAIN_BAUD 9600          // Baud rate used for Serial Monitor Printing
  #define SENSOR_INTERVAL 1       // Minimum interval between sensor collection
  #define AVERAGING_INTERVAL 45   // Minimum Span to average a packet over
  #define MESSAGE_INTERVAL 180    // Minimum interval between Sending Messages to Satellite
  #define PICTURE_INTERVAL 300    // Minimum interval between taking a picture for local storage 
  #define CALLBACK_INTERVALS_COEFFICIENT 2 // The constant by which all intervals all multiplied by for the callback function in the satellite transmission
  #define PACKETS_PER_MESSAGE 4   // Number of Packets in every message to be sent to the sattelite (proportional to max size of message 340, and packet size)
  #define LINES_BUFFER_SIZE 10    // Number of Sensor readings before a write to buffer is necessary
  #define MESSAGE_SIZE 220        // size of each packet in the message * the lineCount of them
  #define MAX_LINES 1000          // Lines used in every sd card file
  #define LED_PIN 5
  #define TEMPHUMID_PIN 7 // See SENSOR_PIN as well
  #define CS_PIN 6                // Camera Pin
  

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
  #include <SD.h>
  #include <SPI.h>
  #include <ArduCAM.h>
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>

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
  IridiumSBD IridiumModem(IridiumSerial);
  #define COLLECT_NUMBER 20       // Ozone data collection range (1-100)
  ArduCAM myCAM(OV2640, CS_PIN);
  Adafruit_SSD1306 display(128, 64, &Wire, -1);
  String oledOutput;
  
// DataPacket Struct Info: Not used as class due to tranmission.
  struct __attribute__((packed)) DataPacket {
    /* Info
      55 Byte Sensor Packet Structure (equal to one line of data)
      The __attribyte__((packed)) command makes sure it is exactly 55 by eliminating padding by the compiler.
      Furthermore, there are 3 bytes included in the final message (1 for complete packet lineCount, and 2 for xor checksum)
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
struct DataTotals {
  uint32_t NO2 = 0, C2H5OH = 0, VOC = 0, CO = 0, ozone = 0;
  float pressure = 0, uv = 0, lux = 0, temperature = 0, humidity = 0;
  uint16_t count = 0;

  void add(const DataPacket& p) {
    NO2 += p.NO2; C2H5OH += p.C2H5OH; VOC += p.VOC; CO += p.CO;
    ozone += p.ozone; pressure += p.pressure; uv += p.uv;
    lux += p.lux; temperature += p.temperature; humidity += p.humidity;
    count++;
  }

  DataPacket average(const DataPacket& latestTimeAndGPS) {
    DataPacket avg = latestTimeAndGPS;
    if (count == 0) return avg;
    avg.NO2         = NO2 / count;
    avg.C2H5OH      = C2H5OH / count;
    avg.VOC         = VOC / count;
    avg.CO          = CO / count;
    avg.ozone       = ozone / count;
    avg.pressure    = pressure / count;
    avg.uv          = uv / count;
    avg.lux         = lux / count;
    avg.temperature = temperature / count;
    avg.humidity    = humidity / count;
    return avg;
  }
  void reset() { *this = DataTotals(); }
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
  void initOled();
  bool hasSecondsPassed(TimeRef&, uint32_t);
  DataPacket collectData();
  int sendSBDMessage();
  void setLights(bool on, int startDelay = 0, int endDelay = 0);
  bool saveSensorData(DataPacket); // Write a packet (row) of time, gps, and sensors to the sd card files.
  bool saveSensorData(const DataPacket* packets, size_t lineCount); // Overload function for multiple packets
  void takePicture(); // take and save picture

// Loop Variables
  bool rtcInitialized = false;
  bool gpsInitialized = false;
  bool is_header = false; // used in camera function
  bool messageReady = false;
  DataTotals totals;
  DataPacket p;
  DataPacket avg_p;
  DataPacket linesBuffer[LINES_BUFFER_SIZE]; // Buffer containing sensor lines to be written to the sd card (useful for reducing writes)
  DataPacket messageBuffer[PACKETS_PER_MESSAGE]; // message with Message Size
  uint8_t message[MESSAGE_SIZE];
  uint32_t lineCount = 0;
  uint8_t lineIndex = 0;
  uint8_t messagePacketCount = 0; // stores the current index to buffer at
  // Timers are updated internally in hasSecondsPassed
  TimeRef last_sensor;  // time of last sensors read
  TimeRef last_image;   // time of last image capture and save
  TimeRef last_average; // time of last averaged packet
  TimeRef last_message; // time of last sent message

void setup() {
  // Initialize serial and I2C
  Serial.begin(MAIN_BAUD); // Set Serial for Printing to 9600 baud as requested
  Wire.begin();
  SD.begin(CS_PIN);
  
  // Initialize all parts
  oledOutput = ""; // initializing the String that will print to OLED
  rtcInitialized = initRTC();
  gpsInitialized = initGPS();
  initGasSensor();
  initOzoneSensor();
  initBarometer();
  initUVSensor();
  initIridium();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);   // Start with LEDs ON
  initOled();

  // Setup Initial times.
  last_message.rtcTime = rtc.now();
  last_message.sysMillis = millis();
  last_average = last_message;
  last_sensor = last_message;
  last_image = last_message;
}

void loop() {
  // Sensor data collection
  setLights(false);

  if (hasSecondsPassed(last_sensor, SENSOR_INTERVAL) && lineIndex < LINES_BUFFER_SIZE) {
      p = collectData();
      linesBuffer[lineIndex++] = p;
      totals.add(p);
      if (lineIndex == LINES_BUFFER_SIZE - 1) {
        saveSensorData(linesBuffer, LINES_BUFFER_SIZE);
        lineIndex = 0;
      }
  }

  setLights(true);

  // Averaging logic
  if (hasSecondsPassed(last_average, AVERAGING_INTERVAL) && !messageReady) {
      DataPacket avg_p = totals.average(p); 
      
      // Only add if message is not already full
      messageBuffer[messagePacketCount++] = avg_p;
      if (messagePacketCount == PACKETS_PER_MESSAGE) {
        messageReady = true; // clearly mark message as ready
      }
      totals.reset();
  }

  setLights(false);

  // Transmission logic (simplified and clear)
  if (messageReady && hasSecondsPassed(last_message, MESSAGE_INTERVAL)) {
    messagePacketCount = 0;
    int status = sendSBDMessage();
    if (status != ISBD_CANCELLED) messageReady = false; // If callback cancelled transmission, new message is ready
  }

  // Image capturing (unchanged)
  if (hasSecondsPassed(last_image, PICTURE_INTERVAL)) {
    takePicture();
  }
  setLights(true);
}

// Initializes OLED and prints to the screen which sensors if any have errors
void initOled(){
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    for(;;);
  }
  delay(2000);

  display.clearDisplay();
  display.setTextSize(2.4);
  display.setTextColor(WHITE);
  display.setCursor(5, 10);
  display.setTextWrap(true);

  if (oledOutput == ""){
    oledOutput = "No errors";
  }

  // Display static text
  display.println(oledOutput);
  display.display();
}

// Initialize Real-Time Clock - returns true if successful
bool initRTC() {
  // Try to initialize RTC
  if (!rtc.begin()) {
    oledOutput += "RTC, ";
    return false;
  }
  
  // Uncomment the line below on first upload to set RTC to computer time
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  
  // Check if RTC lost power and warn (but don't update automatically)
  if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    return true;
  }
  return true;
}

// Initialize GPS - updated for u-blox GPS library
bool initGPS() {
  
  // First try the default address (0x42)
  if (myGNSS.begin(Wire, GPS_I2C_ADDRESS)) {
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
  oledOutput += "GPS, ";
  return false;
}

// Initialize Gas Sensor
bool initGasSensor() {
  if (!gas.begin(Wire, GAS_ADDRESS)) {
    oledOutput += "gas, ";
    return false;
  }
  else {
    gas.begin(Wire, GAS_ADDRESS)
  }
}

// Initialise Ozone sensor
bool initOzoneSensor() {
  if (!ozone.begin(OZONE_ADDRESS)) {
    oledOutput += "ozone, ";
    return false;
  } 
  else {
    ozone.setModes(MEASURE_MODE_PASSIVE);
  }
}

// Initialize Barometer
bool initBarometer() { 
  if (!hp20x.begin()) {
    oledOutput += "baro, ";
    return false;
  }
  else hp20x.begin();
}

// Initialize UV Sensor
bool initUVSensor() {
  if (!ltr390.init()) {
    oledOutput += "UV, "; 
    return false;
  }
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
    oledOutput += "temp, "; 
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
  return true;
}

bool getGPS(DataPacket &packet) {
  if (gpsInitialized) {
    if (myGNSS.isConnected()) {
      // Get positioning data using u-blox methods
      packet.latitude = myGNSS.getLatitude();  // Save latitude to packet
      packet.longitude = myGNSS.getLongitude();  // Save longitude to packet
      packet.altitude = myGNSS.getAltitude();  // Save altitude to packet
      packet.satellites = myGNSS.getSIV();  // Save satellites lineCount to packet
      packet.fixtype = myGNSS.getFixType();  // Save GPS fix type to packet
      return true;

      // Serial.print(latitude / 10000000.0, 6); // Convert to decimal degrees (usually they are scaled by 1e7)
      // Serial.println(longitude / 10000000.0, 6); // convert to meters from millimeters
    } 
  } 
    return false;
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
  int status = am2302.read();
  if (status == 0) {  // Assuming 0 means a successful read
    packet.temperature = am2302.get_Temperature();
    packet.humidity= am2302.get_Humidity();
    return true;
  }
  return false;
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
int sendSBDMessage() {
    // Copy current messageBuffer contents into message (to not conflict with the messageBuffer being updated by the loop)
    memcpy(message, messageBuffer, MESSAGE_SIZE);

    // Send the message
    return IridiumModem.sendSBDBinary(message, MESSAGE_SIZE);
}

// Background Function that is ran behind the scenes while the rockblock is sending a message. Can terminate whole data sendin by returning false.
// For more info see: https://github.com/mikalhart/IridiumSBD/blob/master/extras/IridiumSBD%20Arduino%20Library%20Documentation.pdf
bool ISBDCallback() {
  setLights(false);
  if (hasSecondsPassed(last_sensor, SENSOR_INTERVAL * CALLBACK_INTERVALS_COEFFICIENT) && lineIndex < LINES_BUFFER_SIZE) {
      p = collectData();
      linesBuffer[lineIndex++] = p;
      totals.add(p);
      if (lineIndex == LINES_BUFFER_SIZE - 1) {
        saveSensorData(linesBuffer, LINES_BUFFER_SIZE);
        lineIndex = 0;
      }
  }

  // Averaging logic
  if (hasSecondsPassed(last_average, AVERAGING_INTERVAL * CALLBACK_INTERVALS_COEFFICIENT)) {
      DataPacket avg_p = totals.average(p);
      
      // Only add if message is not already full
      messageBuffer[messagePacketCount++] = avg_p;
      if (messagePacketCount == PACKETS_PER_MESSAGE) {
        messageReady = true; // clearly mark message as ready
      }

      totals.reset();
  }


  // Image capturing (unchanged)
  if (hasSecondsPassed(last_image, PICTURE_INTERVAL * CALLBACK_INTERVALS_COEFFICIENT)) {
    takePicture();
  }

  setLights(true);
  return true;  // Continue current transmission
}

// Turns LED on/off with optional delays. Assumes active means low LED voltage. Delay taken in millis.
void setLights(bool on, int startDelay = 0, int endDelay = 0) {
  delay(startDelay);
  digitalWrite(LED_PIN, (on ? LOW : HIGH));  // LOW = ON (active-low)
  delay(endDelay);
}

// Save one packet per call to SD
bool saveSensorData(DataPacket packet) {
  static int fileIndex = 0;
  static char currentFileName[20] = "";

  // If it's the first call or file is full. Initialise file
  if (lineCount == 0 || lineCount >= MAX_LINES) {
    snprintf(currentFileName, sizeof(currentFileName), "data%d.csv", fileIndex++);
    File dataFile = SD.open(currentFileName, FILE_WRITE);
    if (dataFile) {
      dataFile.println(
        "second,minute,hour,"
        "latitude,longitude,altitude,satellites,fixtype,"
        "NO2,C2H5OH,VOC,CO,ozone,pressure,uv,lux,temperature,humidity"
      );
      dataFile.close();
    }
    lineCount = 0;
  }

  // Open and Append data to file
  File dataFile = SD.open(currentFileName, FILE_WRITE);
  if (!dataFile) return false;
  dataFile.print(packet.second);        dataFile.print(",");
  dataFile.print(packet.minute);        dataFile.print(",");
  dataFile.print(packet.hour);          dataFile.print(",");

  dataFile.print(packet.latitude);      dataFile.print(",");
  dataFile.print(packet.longitude);     dataFile.print(",");
  dataFile.print(packet.altitude);      dataFile.print(",");
  dataFile.print(packet.satellites);    dataFile.print(",");
  dataFile.print(packet.fixtype);       dataFile.print(",");

  dataFile.print(packet.NO2);           dataFile.print(",");
  dataFile.print(packet.C2H5OH);        dataFile.print(",");
  dataFile.print(packet.VOC);           dataFile.print(",");
  dataFile.print(packet.CO);            dataFile.print(",");
  dataFile.print(packet.ozone);         dataFile.print(",");
  dataFile.print(packet.pressure);      dataFile.print(",");
  dataFile.print(packet.uv);            dataFile.print(",");
  dataFile.print(packet.lux);           dataFile.print(",");
  dataFile.print(packet.temperature);   dataFile.print(",");
  dataFile.println(packet.humidity);  // newline

  dataFile.close();
  lineCount++;
}

// Save Multiple Packets (collected during transmission)
bool saveSensorData(const DataPacket* packets, size_t packetCount) {
  for (size_t i = 0; i < packetCount; i++) {
    if (!saveSensorData(packets[i])) {
      return false;
    }
  }
  return true;
}

void takePicture() {
  myCAM.flush_fifo();
  myCAM.clear_fifo_flag();

  #if defined (OV2640_MINI_2MP_PLUS)
  myCAM.OV2640_set_JPEG_size(OV2640_1600x1200);
  #endif

  myCAM.start_capture();

  while (!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK));

  saveImageFromFIFO();

  myCAM.clear_fifo_flag();
}

void saveImageFromFIFO() {
  static uint8_t imageCount = 0;
  uint8_t temp = 0, temp_last = 0;
  uint32_t length = myCAM.read_fifo_length();
  byte buf[256];
  char filename[16];
  File outFile;
  int i = 0;

  if (length == 0 || length >= MAX_FIFO_SIZE) {
    Serial.println(F("Invalid image size."));
    return;
  }

  sprintf(filename, "img%d.jpg", imageCount++);
  outFile = SD.open(filename, FILE_WRITE);
  if (!outFile) {
    return;
  }

  myCAM.CS_LOW();
  myCAM.set_fifo_burst();

  while (length--) {
    temp_last = temp;
    temp = SPI.transfer(0x00);

    if ((temp == 0xD8) && (temp_last == 0xFF)) {
      is_header = true;
      buf[i++] = temp_last;
      buf[i++] = temp;
    } else if ((temp == 0xD9) && (temp_last == 0xFF)) {
      buf[i++] = temp;
      outFile.write(buf, i);
      break;
    } else if (is_header) {
      if (i < 256) {
        buf[i++] = temp;
      } else {
        outFile.write(buf, 256);
        i = 0;
        buf[i++] = temp;
      }
    }
    imageCount++;
  }

  myCAM.CS_HIGH();
  outFile.close();
}