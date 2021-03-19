/***************************
Title:      SnowBot Test Deploy Script
Date:       02Feb2021
Author:     Chris Cosgrove

Components:
-  Adafruit Industries Feather M0 with RFM95 LoRa Radio https://www.adafruit.com/product/3178
-  Adafruit Industries DS3231 Precision RTC FeatherWing https://www.adafruit.com/product/3028
-  Adafruit Industries Adalogger Featherwing https://www.adafruit.com/product/2922.
-  Adafruit Industries Terminal Block Breakout Featherwing https://www.adafruit.com/product/2926
-  SONBEST SHT-10 Mesh-protected Weatherproof Temperature and Humidity Sensor https://www.adafruit.com/product/1298
-  MaxBotix MB7374 HRXL-MaxSonar-WRST7 Ultrasonic Precision Range Finder https://www.maxbotix.com/Ultrasonic_Sensors/MB7374.htm
-  Rock Seven RockBlock Mk II Iridium Satellite Modem http://www.rock7.com/products-rockblock (OPTIONAL)

Comments:
Code draws gratefully on the work of Adam Garbo's Cryologger Automatic Weather Station
https://github.com/adamgarbo/Cryologger_Automatic_Weather_Station

Code samples at 1 hour intervals, hourly averaged data is sent by LoRa to a base station
(node 3), data is sent by RockBlock every six hours.
****************************/

/****************************
Constants
****************************/
#define NODE_STATION   false    // Set if node is just to transmit messages by LoRa
#define BASE_STATION   true     // Set if RockBlock is installed and you wish to listen for LoRa messages
#define DEBUG          false    // Output debug messages to Serial Monitor
#define DEPLOY         true     // Disable debugging messages for deployment
#define DIAGNOSTICS    false    // Output Iridium diagnostic messages to Serial Monitor
#define ROCKBLOCK      true     // If base station define whether RockBlock is used

/****************************
Libraries
****************************/
#include <Arduino.h>            // https://github.com/arduino/ArduinoCore-samd
#include <ArduinoLowPower.h>    // https://github.com/arduino-libraries/ArduinoLowPower
#include <DS3232RTC.h>          // https://github.com/JChristensen/DS3232RTC
#include <Statistic.h>          // https://github.com/RobTillaart/Arduino/tree/master/libraries/Statistic
#include <SdFat.h>              // https://github.com/greiman/SdFat
#include <SPI.h>                // https://www.arduino.cc/en/Reference/SPI
#include <Wire.h>               // https://www.arduino.cc/en/Reference/Wire
#include <RH_RF95.h>            // https://github.com/PaulStoffregen/RadioHead
#include <RHReliableDatagram.h> // https://github.com/PaulStoffregen/RadioHead
#include <SHT1x.h>              // https://github.com/practicalarduino/SHT1x
#include <math.h>               // https://www.nongnu.org/avr-libc/user-manual/group__avr__math.html

#if BASE_STATION
    #include <IridiumSBD.h>     // https://github.com/mikalhart/IridiumSBD
#endif

/***************************
Pin definitions
****************************/
#define VBAT_PIN        A7      // Battery
#define SD_Pin          10      // SD Card
#define RFM95_CS        8       // LoRa Radio
#define RFM95_RST       4       // LoRa Radio
#define RFM95_INT       3       // LoRa Radio
#define WAKE_PIN        6       // RTC Wake Pin - attach DS3231 RTC Interrupt pin to this pin on Feather
#define MB_pwPin        5       // Maxbotix pulseWidth pin
#define MB_sleepPin     11      // Maxbotix sleep pin
#define SHT_clockPin    12      // SHT10
#define SHT_dataPin     13      // SHT10
#define LED_PIN         13      // LED pin

#if BASE_STATION
    #define RB_SLEEP_PIN    9       // Rockblock Sleep Pin
    #define IridiumSerial   Serial1 // RockBlock Serial
#endif

/****************************
Object instantiations
****************************/
SHT1x           sht1x(SHT_dataPin, SHT_clockPin);       // SHT10
SdFat           sd;                                     // File system object
SdFile          file;                                   // Log file
DS3232RTC       myRTC(false);                           // Tell constructor not to initialize the I2C bus
time_t          t;
unsigned long   unixtime;
time_t          alarmTime;
tmElements_t    tm;

#if BASE_STATION
  IridiumSBD      modem(IridiumSerial, RB_SLEEP_PIN);     // RockBlock
#endif

#define RF95_FREQ 915.0                              // Define LoRa frequency
RH_RF95 rf95(RFM95_CS, RFM95_INT);                   // Singleton instance of the radio driver

/****************************
Statistic objects
****************************/
Statistic batteryStats;         // Battery voltage statistics
Statistic humidityStats;        // Humidity statistics
Statistic extTemperatureStats;  // Temperature statistics
Statistic rtcStats;             // Real-time clock statistics
Statistic MaxbotixStats_av;     // Maxbotix average distances
Statistic MaxbotixStats_std;    // Maxbotix std distances
Statistic MaxbotixStats_max;    // Maxbotix max distances
Statistic MaxbotixStats_min;    // Maxbotix min distances
Statistic MaxbotixStats_nan;    // Maxbotix nan samples

/****************************
User defined global variable declarations
****************************/
char*                 node_name             = "s03";        // Node name
unsigned int          node_number           = 3;            // Node number
unsigned int          base_station_number   = 1;            // Number of snow bot for datagram (100 + node)
unsigned int          total_nodes           = 5;            // Total nodes in the network

unsigned int          samplesPerFile        = 8640;         // Maximum samples stored in a file before new log file creation (Default: 30 days * 288 samples per day)
unsigned int          listen                = 45;           // Time in seconds to listen for incoming or sending outgoing LoRa messages

bool                  hourlySend            = true;         // Boolean to set if sending at specific hours instead of a user given interval 
unsigned int          send_hours[] = {0, 6, 12, 18};        // Iridium sending hours
unsigned int          send_hours_size = sizeof(send_hours) / sizeof(long);

unsigned int          sampleInterval        = 3600;         // Sleep duration (in seconds) between data sample acquisitions.
unsigned int          averageInterval       = 1;            // Number of samples to be averaged for each LoRa transmission.
unsigned int          transmitInterval      = 6;            // Number of average intervals to be included in a single transmission (340 byte limit). Each node LoRa
															                              // message averaged sample is 24 bytes, so  if you multiply 24 (no. of bytes) by number of nodes by the
															                              // transmitInterval it should be less than 340 to include all the data.
															                              // E.g. for 5 nodes one average interval (assuming all LoRa messages are received) would equal 120 bytes
															                              // (5*24). Therefore you could have a transmitInterval of 2 to get 2 complete average intervals of data sent
															                              // in a 340 byte RockBlock message.

unsigned int          maxRetransmitCounter  = 0;            // Maximum failed data transmissions to reattempt in a single message (340 byte limit). Default: 10


#define NODE_ADDRESS  node_number           // Node number is local LoRa address
#define BASE_ADDRESS  base_station_number   // Number of station that is LoRa base station w/ RockBlock

/****************************
Datagram set-up
****************************/
#if NODE_STATION
RHReliableDatagram manager(rf95, NODE_ADDRESS);  // Class to manage message delivery and receipt, using the driver declared above
#endif

#if BASE_STATION
RHReliableDatagram manager(rf95, BASE_ADDRESS);  // Class to manage message delivery and receipt, using the driver declared above
#endif

/****************************
Global variables
****************************/
volatile bool           alarmFlag               = false;        // RTC interrupt service routine (ISR) flag
volatile bool           sleepFlag               = false;        // Watchdog Timer Early Warning interrupt flag
volatile bool           loraRxFlag              = false;        // LoRa message received flag
volatile byte           watchdogCounter         = 0;            // Watchdog Timer trigger counter
bool                    ledState                = LOW;          // Flag to toggle LED in blinkLed() function
bool                    logFlag                 = true;         // MicroSD initilization flag
byte                    resetFlag               = 0;            // Watchdog Timer force reset flag
byte                    transmitBuffer[340]     = {};           // RockBLOCK transmission buffer
float                   humidity                = 0.0;          // SHT31 humidity (%)
float                   extTemperature          = 0.0;          // SHT31 temperature (°C)
float                   intTemperature          = 0.0;          // Internal RTC temperature (°C)
unsigned int            distMaxbotix_av         = 0;            // Average distance from Maxbotix sensor to surface (mm)
unsigned int            distMaxbotix_std        = 0;            // Std distance from Maxbotix sensor to surface (mm)
unsigned int            distMaxbotix_max        = 0;            // Max distance from Maxbotix sensor to surface (mm)
unsigned int            distMaxbotix_min        = 0;            // Min distance from Maxbotix sensor to surface (mm)
unsigned int            distMaxbotix_nan        = 0;            // Number of NaN readings in Maxbotix
float                   voltage                 = 0.0;          // Battery voltage in volts (V)
unsigned int            retransmitCounter       = 0;            // RockBLOCK failed data transmission counter
unsigned int            transmitCounter         = 0;            // RockBLOCK transmission interval counter
unsigned int            previousMillis          = 0;            // RockBLOCK callback function timer variable
unsigned int            sampleCounter           = 0;            // Sensor measurement counter
unsigned int            samplesSaved            = 0;            // Log file sample counter
int16_t                 packetnum;                              // LoRa radio packet number
uint8_t                 buf[RH_RF95_MAX_MESSAGE_LEN];           // LoRa radio buffer
uint8_t                 len                    = sizeof(buf);   // LoRa radio buffer length
volatile bool           resFlag                = false;         // LoRa response flag is set to true once response is found
int                     signalQuality          = -1;            // RockBlock Signal quality
int                     err                    = 0;             // RockBlock error
const byte              samplesToAverage       = 10;            // Number of samples to average
char                    fileName[14]           = "s##_0000.csv";// Log file naming convention
uint32_t                period                 = listen*1000UL; // Set up listening loop

#if BASE_STATION
uint8_t rx_reply[] = "ok";
#endif
/****************************
Structure to store and send data 
****************************/
typedef union {
  struct {
    int16_t     node;                 // Node number                              (2_bytes)
    uint32_t    unixtime;             // Date and time in time_t format           (4 bytes)
    int16_t     intTemperature;       // Mean internal temperature (°C)           (2 bytes) (intTemperature * 100)
    int16_t     extTemperature;       // Mean external temperature (°C)           (2 bytes) (extTemperature * 100)
    uint16_t    humidity;             // Mean humidity (%)                        (2 bytes) (humidity * 100)
    uint16_t    distMaxbotix_av;      // Distance from Maxbotix to surface (mm)   (2 bytes)
    uint16_t    distMaxbotix_std;     // Std from Maxbotix to surface (mm)        (2 bytes)
    uint16_t    distMaxbotix_max;     // Max from Maxbotix to surface (mm)        (2 bytes)
    uint16_t    distMaxbotix_min;     // Min from Maxbotix to surface (mm)        (2 bytes)
    uint16_t    distMaxbotix_nan;     // Number of Maxbotix nans                  (2 bytes)
    uint16_t    voltage;              // Minimum battery voltage (mV)             (2 bytes) (voltage * 1000)
  } __attribute__((packed));                                                      // Total = 24 bytes
  uint8_t bytes[24];
} SBDMESSAGE;

SBDMESSAGE tx_message;          // Outgoing LoRa message
SBDMESSAGE rx_message;          // Incoming LoRa message
size_t messageSize = sizeof(tx_message); // Size (in bytes) of data to be stored and transmitted

/****************************
Function Prototypes
****************************/
void alarmIsr();
void readRtc();
void readBattery();
void readTrh();
void readMxBtx();
void createLogFile();
void logData();
void writeTimestamps();
void calculateStatistics();
void blinkLed(byte led, byte flashes, unsigned long interval);
void printDateTime(time_t t);
void printStatistics();
void printUnion();
void printUnionBinary();
void configureWatchdog();
void petDog();
void WDT_Handler();
void talkToRadio();
void talkToSD();

#if NODE_STATION
void LoRa_send();
#endif

#if BASE_STATION
bool ISBDCallback();
void writeBuffer();
void writeBuffer_rx();
void LoRa_receive();
void transmitData();
void printTransmitBuffer();
void ISBDConsoleCallback(IridiumSBD * device, char c);
void ISBDDiagsCallback(IridiumSBD * device, char c);
#endif

/****************************
Set-up
****************************/
void setup()
{
  // Pin assignments
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  // Start the Serial
  Serial.begin(115200);
    //while (!Serial); // Wait for user to open Serial Monitor
  delay(5000); // Delay to allow user to open Serial Monitor

   // Watchdog Timer Configuration
  configureWatchdog();
  
  // I2C Configuration
  Wire.begin();           // Initialize I2C bus
  Wire.setClock(100000);  // Set I2C clock speed to 100kHz

    // Initialise RTC
  myRTC.begin();                                // Initialize the I2C bus
  myRTC.setAlarm(ALM1_MATCH_DATE, 0, 0, 0, 1);  // Initialize the alarms to known values
  myRTC.setAlarm(ALM2_MATCH_DATE, 0, 0, 0, 1);
  myRTC.alarm(ALARM_1);                         // Clear the alarm flags
  myRTC.alarm(ALARM_2);
  myRTC.alarmInterrupt(ALARM_1, false);         // Clear the alarm interrupt flags
  myRTC.alarmInterrupt(ALARM_2, false);
  myRTC.squareWave(SQWAVE_NONE);
  
  // Read the date, time and temperature from the RTC
  readRtc();
  
  // Configure interrupt on INT/SQW pin
  pinMode(WAKE_PIN, INPUT_PULLUP);
  LowPower.attachInterruptWakeup(WAKE_PIN, alarmIsr, FALLING);
  
  // Set RTC alarm
  unsigned int startAlarm_minute = sampleInterval / 60; // Get the minute at which to start the alarm so averaging occurs at a sensible time
  
  #if DEBUG
      myRTC.setAlarm(ALM1_MATCH_SECONDS, 0, 0, 0, 1);     // Set initial alarm to occur at seconds rollover
  #else if DEPLOY
      myRTC.setAlarm(ALM1_MATCH_MINUTES, 0, startAlarm_minute, 0, 1);     // Set initial alarm to occur at minutes rollover (start of new hour)
  #endif
      myRTC.alarm(ALARM_1);                               // Ensure alarm 1 interrupt flag is cleared
      myRTC.alarmInterrupt(ALARM_1, true);                // Enable interrupt output for alarm 1
  
  // Print RTC's alarm date and time
  Serial.print(F("Next alarm: ")); printDateTime(alarmTime);
  
  // Print operating mode
  Serial.print(F("Mode: "));
  #if DEBUG
      Serial.println(F("DEBUG"));
  #else if DEPLOY
      Serial.println(F("DEPLOY"));
  #endif
  
  // Print current date and time
  Serial.print(F("Datetime: ")); printDateTime(myRTC.get());
  
  // Test the LoRa Radio
  talkToRadio();
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(250);

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(100);
  digitalWrite(RFM95_RST, HIGH);
  delay(500);
  
  Serial.println("Feather LoRa RX Test!");
  if (!manager.init())
    Serial.println("LoRa manager init failed");
  Serial.println("LoRa radio init OK!");
  
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }

  rf95.setTxPower(23, false);
  Serial.println("Sleeping radio");
  rf95.sleep();
  Serial.println();
  packetnum = 0;  // packet counter, we increment per xmission

//  rf95.setSignalBandwidth(125000);
//  rf95.setCodingRate4(8);
//  rf95.setSpreadingFactor(12); // 4096
//  Serial.println("SUCCESS\nSet Config to: Bw = 125 kHz, Cr = 4/8, Sf = 4096 chips/symbol");
//  uint16_t loraAckTimeout = 1000;
//  manager.setTimeout(loraAckTimeout);

  // MaxBotix
  pinMode(MB_pwPin, INPUT);
  pinMode(MB_sleepPin, OUTPUT);
  
  // Initialise the SD card
  talkToSD(); // Talk to SD card
    
  if (sd.begin(SD_Pin, SD_SCK_MHZ(4))) {
      Serial.println("SD card initialized.");
      logFlag = true;
      createLogFile(); // Create new log file
      blinkLed(LED_PIN, 20, 100);
  }
  else {
      Serial.println("Warning: Unable to initialize SD card!");
      logFlag = false;  // Disable data logging
      blinkLed(LED_BUILTIN, 20, 100);
  }
  
#if ROCKBLOCK
    // Start the serial port connected to the satellite modem
  IridiumSerial.begin(19200);
  
    // Setup the Iridium modem
  modem.setPowerProfile(IridiumSBD::DEFAULT_POWER_PROFILE); // Assume battery power
  modem.adjustSendReceiveTimeout(180); // Default = 300 seconds
  modem.adjustATTimeout(20);
  
  if (modem.begin() != ISBD_SUCCESS)
  {
    Serial.println("Couldn't begin modem operations.");
    exit(0);
  }

    // Check the signal quality (optional)
  signalQuality = -1;
  err = modem.getSignalQuality(signalQuality);
  if (err != 0)
  {
    Serial.print("SignalQuality failed: error ");
    Serial.println(err);
  }
  Serial.print("Signal quality is ");
  Serial.println(signalQuality);

    // Put modem to sleep
  Serial.println("Putting modem to sleep.");
  err = modem.sleep();
#endif
  
  // Print set-up complete statement
  Serial.print("Set up complete for SnowBot #"); Serial.println(node);
  
  // Blink LED to indicate setup has completed
  blinkLed(LED_PIN, 10, 100);
}

/****************************
Loop
****************************/
void loop()
{

  // Check if alarm interrupt service routine was triggered
  if (alarmFlag) {

    // Check to see if the alarm flag is set (also resets the flag if set)
    if (myRTC.alarm(ALARM_1)) {

      // Clear sleep flag
      sleepFlag = false; 

      // Read the date, time and temperature from the RTC
      readRtc();

      // Increment the sample counter
      sampleCounter++;

      // Pet the Watchdog Timer
      petDog();

      // Print date and time
      Serial.print(F("Alarm trigger: ")); printDateTime(t);

      // Perform measurements
      readBattery();    // Read battery voltage
      readTrh();        // Read temperature and humidity
      readMxBtx();      // Read Maxbotix Distance sensor
      
      // Log data to microSD card
      logData();

      // Perform statistics on measurements
      if (sampleCounter == averageInterval) {
        printStatistics();
        calculateStatistics();
      
#if NODE_STATION
        // Send LoRa data
        // Start loop to keep sending data for given period until acknowledgement received
        uint32_t tStart = millis();
        uint32_t tEnd = tStart;
        while ((tEnd - tStart) <= period){
          LoRa_send();
          petDog(); // Pet dog after each attempt to send
              // Exit loop if acknowledgement of receipt is received
          if (loraRxFlag == true) {
            break;
          }
          tEnd = millis();
        }

        // Sleep the radio when done
        rf95.sleep();

        // Clear data stored in tx_message
        memset(tx_message.bytes, 0x00, sizeof(tx_message));
        
        // Reset LoRa flag
        loraRxFlag = false;
#endif
      
#if BASE_STATION
        // Clear the buffer if there's too much data in it
        if ((transmitCounter*messageSize) > 340) {
          transmitCounter = 0;
          memset(transmitBuffer, 0x00, sizeof(transmitBuffer));   // Clear transmitBuffer array   
        }

        // If using the set hour sending scheme, clear the buffer to just get most recent data
        if (hourlySend == true) {
          transmitCounter = 0;
          memset(transmitBuffer, 0x00, sizeof(transmitBuffer));   // Clear transmitBuffer array 
        }
          
        // Write local measurements to buffer
        writeBuffer();
        
        // Start timed loop to receive messages
        Serial.print("Listening for messages for "); Serial.print(listen);
        Serial.println(" seconds");
          
        uint32_t tStart = millis();
        uint32_t tEnd = tStart;
        while ((tEnd - tStart) <= period){
              LoRa_receive(); // Listening function
              petDog();
              tEnd = millis();
        }
  
        // Sleep the radio when done
        rf95.sleep();
        
        // Transmit data if hour matches
        if (hourlySend == true) {
          for (int i = 0; i < send_hours_size; i++){
            if (send_hours[i] == hour(t)){
#if DEBUG
              Serial.println("Pretend transmission!");
              printTransmitBuffer();
#endif          
#if ROCKBLOCK
              transmitData();
              i = send_hours_size; // force end of array check
#endif
            }
          }
        }

        // Transmit data if using a counter
        else {
          if (transmitCounter >= (transmitInterval*total_nodes)) {
#if DEBUG
            Serial.println("Pretend transmission!");
            printTransmitBuffer();
#endif          
#if ROCKBLOCK
            transmitData();
#endif
            transmitCounter = 0;
          }
        }

#endif BASESTATION

        // Reset sample counter
        sampleCounter = 0; // Reset sample counter
     }

      // Set the RTC alarm
      alarmTime = t + sampleInterval;   // Calculate next alarm
    
      // Check if alarm was set in the past
      if (alarmTime <= myRTC.get()) {
        Serial.print(F("Warning! Alarm set in the past!"));
        t = myRTC.get(); // Read current date and time
        alarmTime = t + sampleInterval; // Calculate next alarm
        myRTC.setAlarm(ALM1_MATCH_MINUTES, 0, 0, 0, 1); // Set alarm to occur at minutes rolloever
      }
      else {
        myRTC.setAlarm(ALM1_MATCH_DATE, 0, minute(alarmTime), hour(alarmTime), day(alarmTime)); // Set alarm
      }
      myRTC.alarm(ALARM_1); // Ensure alarm 1 interrupt flag is cleared
    
      Serial.print(F("Next alarm: ")); printDateTime(alarmTime);
      }
      alarmFlag = false; // Clear alarm interrupt service routine flag
  }
  sleepFlag = true; // Clear sleep flag

#if DEBUG
  blinkLed(LED_BUILTIN, 1, 500);

#else if DEPLOY
  blinkLed(LED_BUILTIN, 1, 10);
  LowPower.deepSleep(); // Enter deep sleep
#endif
}

/*******************************************************
Functions
****************************/

// RTC interrupt service routine (ISR)
void alarmIsr() {
  alarmFlag = true;
}

// Measure internal temperature from DS3231 RTC
void readRtc() {

  myRTC.read(tm);     // Read current date and time
  t = makeTime(tm);   // Change the tm structure into time_t (seconds since epoch)
  unixtime = t;       // Alarm 1 trigger time
  intTemperature = myRTC.temperature() / 4.0; // Internal DS3231 temperature

  // Add to statistics object
  rtcStats.add(intTemperature);

  // Write data to union
  tx_message.unixtime = unixtime;

}

// Measure battery voltage from 330kOhm/1MOhm divider
void readBattery() {
#if DEBUG
  unsigned int loopStartTime = millis();
#endif
  voltage = 0.0;
  for (byte i = 0; i < 10; i++)
  {
    voltage += analogRead(VBAT_PIN);
    delay(10);
  }
  voltage /= 10;
  voltage *= 2;     // Multiply back
  voltage *= 3.3;   // Multiply by 3.3V reference voltage
  voltage /= 1024;  // Convert to voltage

  // Add to statistics object
  batteryStats.add(voltage);

  Serial.print("Voltage: "); Serial.println(voltage);

#if DEBUG
  unsigned int batteryLoopTime = millis() - loopStartTime;
  Serial.print("readBattery() function execution: "); Serial.print(batteryLoopTime); Serial.println(F(" ms"));
#endif
}

// Read temperature and humidity from Davis Instruments 6830
void readTrh() {
  // Pull SHT10 pins high
  digitalWrite(SHT_dataPin, HIGH);
  digitalWrite(SHT_clockPin, HIGH);
  delay(50);

  // Read sensor
  extTemperature = sht1x.readTemperatureC();
  humidity = sht1x.readHumidity();

  // Add to statistics object
  extTemperatureStats.add(extTemperature);
  humidityStats.add(humidity);

  Serial.print("Temperature: "); Serial.print(extTemperature); Serial.println(" ºC");
  Serial.print("Humidity: "); Serial.print(humidity); Serial.println("%");

  // Pull SHT10 pins high
  digitalWrite(SHT_dataPin, LOW);
  digitalWrite(SHT_clockPin, LOW);
  delay(50);
}

// Read Maxbotix distance to surface
void readMxBtx() {
  // Wake sensor
  digitalWrite(MB_sleepPin, HIGH);
  delay(100);
  
  // Create a temporary Statistic array to hold the maxbotix measurements
  Statistic Maxbotix;

  // Create temporary variables
  unsigned int z, z_av, z_std, z_max, z_min, z_nan;
  z = 0;
  z_av = 0;
  z_std = 0;
  z_max = 0;
  z_min = 0;
  z_nan = 0;
  
  // Get 30 z readings in mm, filtering out reading 50 mm
  // above/below sensor minumum/maximum readings
  for(byte i = 0; i < 30; i++) {
    z = pulseIn(MB_pwPin, HIGH); // Read distance to snow surface
    
    if (z > 550 && z < 4950) { // Filter readings
      Maxbotix.add(z); // Add good readings to stats array
    }
    else {
      z_nan += 1; // Count bad readings
    }
    delay(100); // Delay 0.1 secs between readings
  }

  // Get stats from the Maxbotix array
  z_av = Maxbotix.average(), 0;
  z_std = Maxbotix.pop_stdev(), 0;
  z_max = Maxbotix.maximum(), 0;
  z_min = Maxbotix.minimum(), 0;
    
  // Deal with issue of a maximum long number in the instance of no
  // readings within filtered range
  if (z_av > 5000) {
    z_av = 0;
  }
  if (z_std > 5000) {
    z_std = 0;
  }
  if (z_max > 5000) {
  z_max = 0;
  }
  if (z_min > 5000) {
  z_min = 0;
  }
  
  // Add sample stats to global arrays
  MaxbotixStats_av.add(z_av);
  MaxbotixStats_std.add(z_std);
  MaxbotixStats_max.add(z_max);
  MaxbotixStats_min.add(z_min);
  MaxbotixStats_nan.add(z_nan);

  // Add to sample variables
  distMaxbotix_av  = z_av;
  distMaxbotix_std = z_std;
  distMaxbotix_max = z_max;
  distMaxbotix_min = z_min;
  distMaxbotix_nan = z_nan;

  // Clear local array
  Maxbotix.clear();

  Serial.print("Distance: "); Serial.print(z_av); Serial.println(" mm");
  
  // Sleep sensor
  digitalWrite(MB_sleepPin, LOW);
  delay(100);
  
}

// Create log file
void createLogFile() {

  // Talk to SD card
  talkToSD();
    
  // Check if logging is enabled
  if (logFlag == true) {
    if (file.isOpen())
      file.close();

    // Select a unique log file name
    for (unsigned int i = 0; i < 9999; i++) {
      snprintf(fileName, sizeof(fileName), "%s_%04d.csv", node_name, i);
      // If O_CREAT and O_EXCL are set, open() will fail if the file already exists
      if (file.open(fileName, O_CREAT | O_EXCL | O_WRITE)) {
        break; // Break out of loop upon successful file creation
      }
    }

    if (!file.isOpen()) {
      Serial.println(F("Unable to open file"));
    }

    // Read RTC date and time
    myRTC.read(tm);

    // Set the file's creation date and time
    if (!file.timestamp(T_CREATE, (tm.Year + 1970), tm.Month, tm.Day, tm.Hour, tm.Minute, tm.Second)) {
      Serial.println("Set create time failed");
    }

    // Write header to file
    file.println("NODE,UNIXTIME,VBAT,T_Int,T_Ext,RH,Z_av,Z_std,Z_max,Z_min,Z_nan");

    // Close log file
    file.close();

    Serial.println(F("New log file created: "));
    Serial.println(fileName);
  }
}

// Write data to log file
void logData() {
    
  // Talk to SD card
  talkToSD();
  
  // Check if logging is enabled
  if (logFlag == true) {
    // Check that maximum file sample limit has not been exceeded
    if (samplesSaved >= samplesPerFile) {
      createLogFile();
      samplesSaved = 0;
    }

    // Write to microSD card
    if (file.open(fileName, O_APPEND | O_WRITE)) {
      samplesSaved++;   //  Increment sample count of current file
      file.print(node);
      file.write(",");
      file.print(unixtime);
      file.write(",");
      file.print(voltage);
      file.write(",");
      file.print(intTemperature);
      file.write(",");
      file.print(extTemperature);
      file.write(",");
      file.print(humidity);
      file.write(",");
      file.print(distMaxbotix_av);
      file.write(",");
      file.print(distMaxbotix_std);
      file.write(",");
      file.print(distMaxbotix_max);
      file.write(",");
      file.print(distMaxbotix_min);
      file.write(",");
      file.println(distMaxbotix_nan);
      writeTimestamps();
      file.close();

#if DEBUG
      Serial.print(node_name);
      Serial.print(",");
      Serial.print(unixtime);
      Serial.print(",");
      Serial.print(voltage);
      Serial.print(",");
      Serial.print(intTemperature);
      Serial.print(",");
      Serial.print(extTemperature);
      Serial.print(",");
      Serial.print(humidity);
      Serial.write(",");
      Serial.print(distMaxbotix_av);
      Serial.write(",");
      Serial.print(distMaxbotix_std);
      Serial.write(",");
      Serial.print(distMaxbotix_max);
      Serial.write(",");
      Serial.print(distMaxbotix_min);
      Serial.write(",");
      Serial.println(distMaxbotix_nan);
      blinkLed(LED_PIN, 2, 100);
#endif
    }
    else {
      Serial.println(F("Unable to open file"));
      logFlag = false;
    }
  }
}

// Log file write and access timestamps
void writeTimestamps() {
    
  // Talk to SD card
  talkToSD();

  // Read RTC date and time
  myRTC.read(tm);

  // Set the file's last write/modification date and time
  if (!file.timestamp(T_WRITE, (tm.Year + 1970), tm.Month, tm.Day, tm.Hour, tm.Minute, tm.Second)) {
    Serial.println(F("Set write time failed"));
  }

  // Set the file's last access date and time
  if (!file.timestamp(T_ACCESS, (tm.Year + 1970), tm.Month, tm.Day, tm.Hour, tm.Minute, tm.Second)) {
    Serial.println(F("Set access time failed"));
  }
}

// Calculate statistics and clear objects
void calculateStatistics() {

  // Write data to union
  tx_message.node = node;
  tx_message.voltage = batteryStats.minimum() * 1000;              // Minimum battery voltage (mV)
  tx_message.extTemperature = extTemperatureStats.average() * 100; // Mean temperature (°C)
  tx_message.humidity = humidityStats.average() * 100;             // Mean humidity (%)
  tx_message.intTemperature = rtcStats.average() * 100;            // Mean RTC temperature (°C)
  tx_message.distMaxbotix_av = MaxbotixStats_av.average();         // Mean Maxbotix distance
  tx_message.distMaxbotix_std = MaxbotixStats_std.average();       // Mean Maxbotix std
  tx_message.distMaxbotix_max = MaxbotixStats_max.maximum();       // Max Maxbotix max
  tx_message.distMaxbotix_min = MaxbotixStats_min.minimum();       // Min Maxbotix min
  tx_message.distMaxbotix_nan = MaxbotixStats_nan.average();       // Mean MaxBotix nan
    

  // Clear statistics objects
  batteryStats.clear();
  humidityStats.clear();
  rtcStats.clear();
  extTemperatureStats.clear();
  MaxbotixStats_av.clear();
  MaxbotixStats_std.clear();
  MaxbotixStats_max.clear();
  MaxbotixStats_min.clear();
  MaxbotixStats_nan.clear();
    
}

// Blink LED
void blinkLed(byte led, byte flashes, unsigned long interval) {
  petDog();
  byte i = 0;
  while (i >= flashes) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      if (ledState == LOW) {
        ledState = HIGH;
      }
      else {
        ledState = LOW;
      }
      digitalWrite(led, ledState);
      i++;
    }
  }
  digitalWrite(led, LOW);
}

// Print current time and date
void printDateTime(time_t t) {
  Serial.print((day(t) < 10) ? "0" : ""); Serial.print(day(t), DEC); Serial.print('/');
  Serial.print((month(t) < 10) ? "0" : ""); Serial.print(month(t), DEC); Serial.print('/');
  Serial.print(year(t), DEC); Serial.print(' ');
  Serial.print((hour(t) < 10) ? "0" : ""); Serial.print(hour(t), DEC); Serial.print(':');
  Serial.print((minute(t) < 10) ? "0" : ""); Serial.print(minute(t), DEC); Serial.print(':');
  Serial.print((second(t) < 10) ? "0" : ""); Serial.println(second(t), DEC);
}

// Print statistics
void printStatistics() {
  Serial.println();
  Serial.println(F("Statistics"));
  Serial.println(F("============================================================================"));
  Serial.print(F("Voltage\t\t"));
  Serial.print(F("Samples: ")); Serial.print(batteryStats.count());
  Serial.print(F("\tMin: "));   Serial.print(batteryStats.minimum());
  Serial.print(F("\tMax: ")); Serial.print(batteryStats.maximum());
  Serial.print(F("\tMean: ")); Serial.println(batteryStats.average());
  Serial.print(F("TemperatureInt\t"));
  Serial.print(F("Samples: ")); Serial.print(rtcStats.count());
  Serial.print(F("\tMin: ")); Serial.print(rtcStats.minimum());
  Serial.print(F("\tMax: ")); Serial.print(rtcStats.maximum());
  Serial.print(F("\tMean: ")); Serial.println(rtcStats.average());
  Serial.print(F("TemperatureExt\t"));
  Serial.print(F("Samples: ")); Serial.print(extTemperatureStats.count());
  Serial.print(F("\tMin: ")); Serial.print(extTemperatureStats.minimum());
  Serial.print(F("\tMax: ")); Serial.print(extTemperatureStats.maximum());
  Serial.print(F("\tMean: ")); Serial.println(extTemperatureStats.average());
  Serial.print(F("Humidity\t"));
  Serial.print(F("Samples: ")); Serial.print(humidityStats.count());
  Serial.print(F("\tMin: ")); Serial.print(humidityStats.minimum());
  Serial.print(F("\tMax: ")); Serial.print(humidityStats.maximum());
  Serial.print(F("\tMean: ")); Serial.println(humidityStats.average());
  Serial.print(F("Maxbotix Dist\t"));
  Serial.print(F("Samples: ")); Serial.print(MaxbotixStats_av.count());
  Serial.print(F("\tMin: ")); Serial.print(MaxbotixStats_min.minimum());
  Serial.print(F("\tMax: ")); Serial.print(MaxbotixStats_max.maximum());
  Serial.print(F("\tMean: ")); Serial.print(MaxbotixStats_av.average());
  Serial.print(F("\tStd: ")); Serial.print(MaxbotixStats_std.average());
  Serial.print(F("\tNaN: ")); Serial.println(MaxbotixStats_nan.average());
    
}

// Print union/structure
void printUnion() {
  Serial.println();
  Serial.println(F("Union/structure"));
  Serial.println(F("==================================="));
  Serial.print(F("node:\t\t")); Serial.println(tx_message.node);
  Serial.print(F("unixtime:\t\t")); Serial.println(tx_message.unixtime);
  Serial.print(F("intTemperature:\t")); Serial.println(tx_message.intTemperature);
  Serial.print(F("extTemperature:\t\t")); Serial.println(tx_message.extTemperature);
  Serial.print(F("humidity:\t\t")); Serial.println(tx_message.humidity);
  Serial.print(F("Maxbotix Av. Dist.:\t\t")); Serial.println(tx_message.distMaxbotix_av);
  Serial.print(F("Maxbotix Std. Dist.:\t\t")); Serial.println(tx_message.distMaxbotix_std);
  Serial.print(F("Maxbotix Max. Dist.:\t\t")); Serial.println(tx_message.distMaxbotix_max);
  Serial.print(F("Maxbotix Min. Dist.:\t\t")); Serial.println(tx_message.distMaxbotix_min);
  Serial.print(F("Maxbotix Nan. Count:\t\t")); Serial.println(tx_message.distMaxbotix_nan);
  Serial.print(F("voltage:\t\t")); Serial.println(tx_message.voltage);
}

// Print contents of union/structure
void printUnionBinary() {
  Serial.println();
  Serial.println(F("Union/structure"));
  Serial.println(F("========================="));
  Serial.println(F("Byte\tHex\tBinary"));
  for (unsigned int i = 0; i < sizeof(tx_message); ++i) {
    Serial.print(i);
    Serial.print("\t");
    Serial.print(tx_message.bytes[i], HEX);
    Serial.print("\t");
    Serial.println(tx_message.bytes[i], BIN);
  }
}

// Set up the WDT to perform a system reset if the loop() blocks for more than 16 seconds
void configureWatchdog() {
  // Set up the generic clock (GCLK2) used to clock the watchdog timer at 1.024kHz
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(4) |            // Divide the 32.768kHz clock source by divisor 32, where 2^(4 + 1): 32.768kHz/32=1.024kHz
                    GCLK_GENDIV_ID(2);              // Select Generic Clock (GCLK) 2
  while (GCLK->STATUS.bit.SYNCBUSY);                // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_DIVSEL |          // Set to divide by 2^(GCLK_GENDIV_DIV(4) + 1)
                     GCLK_GENCTRL_IDC |             // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |           // Enable GCLK2
                     GCLK_GENCTRL_SRC_OSCULP32K |   // Set the clock source to the ultra low power oscillator (OSCULP32K)
                     GCLK_GENCTRL_ID(2);            // Select GCLK2
  while (GCLK->STATUS.bit.SYNCBUSY);                // Wait for synchronization

  // Feed GCLK2 to WDT (Watchdog Timer)
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |           // Enable GCLK2 to the WDT
                     GCLK_CLKCTRL_GEN_GCLK2 |       // Select GCLK2
                     GCLK_CLKCTRL_ID_WDT;           // Feed the GCLK2 to the WDT
  while (GCLK->STATUS.bit.SYNCBUSY);                // Wait for synchronization

  WDT->EWCTRL.bit.EWOFFSET = 0xA;                   // Set the Early Warning Interrupt Time Offset to 8 seconds //REG_WDT_EWCTRL = WDT_EWCTRL_EWOFFSET_8K;
  WDT->INTENSET.bit.EW = 1;                         // Enable the Early Warning interrupt //REG_WDT_INTENSET = WDT_INTENSET_EW;
  WDT->CONFIG.bit.PER = 0xB;                        // Set the WDT reset timeout to 16 seconds //REG_WDT_CONFIG = WDT_CONFIG_PER_16K;
  WDT->CTRL.bit.ENABLE = 1;                         // Enable the WDT in normal mode //REG_WDT_CTRL = WDT_CTRL_ENABLE;
  while (WDT->STATUS.bit.SYNCBUSY);                 // Await synchronization of registers between clock domains

  // Configure and enable WDT interrupt
  NVIC_DisableIRQ(WDT_IRQn);
  NVIC_ClearPendingIRQ(WDT_IRQn);
  NVIC_SetPriority(WDT_IRQn, 0);                    // Top priority
  NVIC_EnableIRQ(WDT_IRQn);
}

// Pet the Watchdog Timer
void petDog() {
  watchdogCounter = 0;              // Clear Watchdog Timer trigger counter
  WDT->CLEAR.bit.CLEAR = 0xA5;      // Clear the Watchdog Timer and restart time-out period //REG_WDT_CLEAR = WDT_CLEAR_CLEAR_KEY;
  while (WDT->STATUS.bit.SYNCBUSY); // Await synchronization of registers between clock domains
}

// Watchdog Timer interrupt service routine
void WDT_Handler() {
  // Check if system is asleep
  if (sleepFlag) {
    // Permit a number Watchdog Timer triggers before forcing system reset.
    if (watchdogCounter < 10) {
      WDT->INTFLAG.bit.EW = 1;          // Clear the Early Warning interrupt flag //REG_WDT_INTFLAG = WDT_INTFLAG_EW;
      WDT->CLEAR.bit.CLEAR = 0xA5;      // Clear the Watchdog Timer and restart time-out period //REG_WDT_CLEAR = WDT_CLEAR_CLEAR_KEY;
      while (WDT->STATUS.bit.SYNCBUSY); // Await synchronization of registers between clock domains
    }
  }
  else {
    while (true);                     // Force Watchdog Timer to reset the system
  }
}

// Set-up to talk to LoRA
void talkToRadio(){
  digitalWrite(SD_Pin, HIGH);
  digitalWrite(RFM95_CS, LOW);
  delay(1);
}

// Set-up to talk to SD
void talkToSD(){
  digitalWrite(RFM95_CS, HIGH);
  digitalWrite(SD_Pin, LOW);
  delay(1);
}

// Send LoRa line
#if NODE_STATION
void LoRa_send(){
  unsigned int loopStartTime = millis();
  
  talkToRadio();
  packetnum++;
  
  #if DEBUG
  Serial.println("Transmitting via LoRa"); // Send a message to rf95_server
  #endif

  // Send a message to manager_server
  if (manager.sendtoWait((uint8_t *)&tx_message, sizeof(tx_message), BASE_ADDRESS))
  {
    // Now wait for a reply from the server
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (manager.recvfromAckTimeout(buf, &len, 4000, &from))
    {
      Serial.print("got reply from base station :");
      Serial.print(from, HEX);
      Serial.print(": ");
      Serial.println((char*)buf);
      loraRxFlag = true;
    }
    else
    {
      Serial.println("No reply, is the base station running?");
    }
  }
  else
    Serial.println("sendtoWait failed");

  Serial.print("RSSI = "); Serial.println(rf95.lastRssi());
  
  unsigned int loraLoopTime = millis() - loopStartTime;
  Serial.print("LoRa_send() function execution: "); Serial.print(loraLoopTime); Serial.println(F(" ms"));
}
#endif

#if BASE_STATION
// Write local data to RockBlock buffer
void writeBuffer() {
  
  // Increment transmit counter
  transmitCounter++;
  
  // Concatenate current tx_message with existing tx_message(s) stored in transmit buffer
  memcpy(transmitBuffer + (sizeof(tx_message) * (transmitCounter + (retransmitCounter * transmitInterval) - 1)), tx_message.bytes, sizeof(tx_message)); // Copy tx_message to transmit buffer

  // Print data to Serial Monitor
//  printUnion();             // Print data stored in union
//  printUnionBinary();       // Print data stored in union in binary format
//  printTransmitBuffer();    // Print data stored transmit buffer array in binary format

  // Clear data stored in union
  memset(tx_message.bytes, 0x00, sizeof(tx_message));
}

// Write received data from LoRa to RockBlock buffer
void writeBuffer_rx() {

  // Increment transmit counter
  transmitCounter++;
  
  // Concatenate current tx_message with existing tx_message(s) stored in transmit buffer
  memcpy(transmitBuffer + (sizeof(rx_message) * (transmitCounter + (retransmitCounter * transmitInterval) - 1)), rx_message.bytes, sizeof(rx_message)); // Copy rx_message to transmit buffer

  // Print data to Serial Monitor
//  printUnion();             // Print data stored in union
//  printUnionBinary();       // Print data stored in union in binary format
//  printTransmitBuffer();    // Print data stored transmit buffer array in binary format

}

// Print contents of transmiff buffer array
void printTransmitBuffer() {
  Serial.println();
  Serial.println(F("Transmit buffer"));
  Serial.println(F("========================="));
  Serial.println(F("Byte\tHex\tBinary"));
  for (unsigned int i = 0; i < 340; i++) {
    Serial.print(i);
    Serial.print("\t");
    Serial.print(transmitBuffer[i], HEX);
    Serial.print("\t");
    Serial.println(transmitBuffer[i], BIN);
  }
}

// Transmit data via the RockBLOCK 9602 transceiver
void transmitData() {

  // Start loop timer
  unsigned long loopStartTime = millis();
  unsigned int err;

  // Start the serial power connected to the RockBLOCK modem
  IridiumSerial.begin(19200);

  // Begin satellite modem operation
  Serial.println(F("Starting modem..."));
  err = modem.begin();
  if (err == ISBD_SUCCESS) {

    /*
        // Test the signal quality
        int signalQuality = -1;
        err = modem.getSignalQuality(signalQuality);
        if (err != ISBD_SUCCESS) {
          Serial.print(F("SignalQuality failed: error "));
          Serial.println(err);
          return;
        }
      Serial.print(F("On a scale of 0 to 5, signal quality is currently: "));
      Serial.println(signalQuality);
    */

    // Transmit and receieve data in binary format
    Serial.println(F("Attempting to transmit data..."));
    err = modem.sendSBDBinary(transmitBuffer, (sizeof(tx_message) * (transmitCounter + (retransmitCounter * transmitInterval))));

    // Check if transmission was successful
    if (err == ISBD_SUCCESS) {
      retransmitCounter = 0;
      memset(transmitBuffer, 0x00, sizeof(transmitBuffer)); // Clear transmit buffer array
    }
    else {
      Serial.print(F("Transmission failed: error "));
      Serial.println(err);
    }
  }
  else {
    Serial.print(F("Begin failed: error "));
    Serial.println(err);
    if (err == ISBD_NO_MODEM_DETECTED) {
      Serial.println(F("Warning: No modem detected. Please check wiring."));
    }
    return;
  }

  // If transmission or modem begin fails
  if (err != ISBD_SUCCESS) {
    retransmitCounter++;

    // Reset counter if retransmit counter is exceeded
    if (retransmitCounter >= maxRetransmitCounter) {
      retransmitCounter = 0;
      memset(transmitBuffer, 0x00, sizeof(transmitBuffer));   // Clear transmitBuffer array
    }
  }

  // Power down the modem
  Serial.println(F("Putting the RockBLOCK to sleep."));
  err = modem.sleep();
  if (err != ISBD_SUCCESS) {
    Serial.print(F("Sleep failed: error "));
    Serial.println(err);
  }

  // Close the serial port connected to the RockBLOCK modem
  IridiumSerial.end();

  // Stop loop timer
  unsigned long loopEndTime = millis() - loopStartTime;
  Serial.print(F("transmitData() function execution: ")); Serial.print(loopEndTime); Serial.println(F(" ms"));

  Serial.print(F("retransmitCounter: ")); Serial.println(retransmitCounter);
}

// RockBLOCK callback function
bool ISBDCallback() {  // This function can be repeatedly called during data transmission or GPS signal acquisitionn
#if DEBUG
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, (millis() / 1000) % 2 == 1 ? HIGH : LOW);
  pinMode(LED_BUILTIN, INPUT);
#endif

  unsigned int currentMillis = millis();
  if (currentMillis - previousMillis >= 2000) {
    previousMillis = currentMillis;
    readBattery(); // Measure battery voltage
    petDog();  // Pet the dog
  }
  return true;
}

// Callback to sniff the conversation with the Iridium modem
void ISBDConsoleCallback(IridiumSBD * device, char c) {
#if DIAGNOSTICS
Serial.write(c);
#endif
}


// Callback to to monitor library's run state
void ISBDDiagsCallback(IridiumSBD * device, char c) {
#if DIAGNOSTICS
Serial.write(c);
#endif
}

// Function to listen to messages and write to datafile
void LoRa_receive(){

  // Volatile boolean for if a message is received
  volatile bool rxFlag = false;
  
  talkToRadio();

  if (manager.available())
  {
    // Wait for a message addressed to us from the client
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (manager.recvfromAck(buf, &len, &from))
    {
      Serial.print("Got data from SnowBot #");
      Serial.print(from, HEX);
      Serial.print(": ");
      Serial.println((char*)buf);
      rxFlag = true; //Set rxFlag to true
      
      // Send a reply back to the originator client
      if (!manager.sendtoWait(rx_reply, sizeof(rx_reply), from))
      {
          Serial.println("sendtoWait failed");
          rxFlag = false; //Set rxFlag to false if sendtoWait fails
      }
    }
  }

  if (rxFlag) {
    // Add received buf into rx_message
    memcpy(&rx_message, buf, sizeof(rx_message)); // Copy received into rx_message
    
    // Write received rx message to the RockBlock buffer
    writeBuffer_rx();
  
    // Write received message to SD file
    talkToSD(); // Talk to SD card
  
    // Check if logging is enabled
    if (logFlag == true) {
      // Check that maximum file sample limit has not been exceeded
      if (samplesSaved >= samplesPerFile) {
      createLogFile();
      samplesSaved = 0;
      }
  
      // Write to microSD card
      if (file.open(fileName, O_APPEND | O_WRITE)) {
        samplesSaved++;   //  Increment sample count of current file
        file.print(rx_message.node);
        file.write(",");
        file.print(rx_message.unixtime);
        file.write(",");
        file.print(rx_message.voltage/1000);
        file.write(",");
        file.print(rx_message.intTemperature/100);
        file.write(",");
        file.print(rx_message.extTemperature/100);
        file.write(",");
        file.print(rx_message.humidity/100);
        file.write(",");
        file.print(rx_message.distMaxbotix_av);
        file.write(",");
        file.print(rx_message.distMaxbotix_std);
        file.write(",");
        file.print(rx_message.distMaxbotix_max);
        file.write(",");
        file.print(rx_message.distMaxbotix_min);
        file.write(",");
        file.println(rx_message.distMaxbotix_nan);
        writeTimestamps();
        file.close();
        
        #if DEBUG
        Serial.print(rx_message.node);
        Serial.print(",");
        Serial.print(rx_message.unixtime);
        Serial.print(",");
        Serial.print(rx_message.voltage);
        Serial.print(",");
        Serial.print(rx_message.intTemperature);
        Serial.print(",");
        Serial.print(rx_message.extTemperature);
        Serial.print(",");
        Serial.print(rx_message.humidity);
        Serial.write(",");
        Serial.print(rx_message.distMaxbotix_av);
        Serial.write(",");
        Serial.print(rx_message.distMaxbotix_std);
        Serial.write(",");
        Serial.print(rx_message.distMaxbotix_max);
        Serial.write(",");
        Serial.print(rx_message.distMaxbotix_min);
        Serial.write(",");
        Serial.println(rx_message.distMaxbotix_nan);
        blinkLed(LED_PIN, 2, 100);
        #endif
      }
      else {
        Serial.println(F("Unable to open file"));
        logFlag = false;
      }
    }
  }
  
  // Clear data stored in rx_message
  memset(rx_message.bytes, 0x00, sizeof(rx_message));

  // Clear rxFlag
  rxFlag = false;
}
#endif
