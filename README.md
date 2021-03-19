## SnowBot overview

Meteorological data scarcity in high-latitude and high-altitude areas is a significant challenge in understanding the impacts of a warming climate on seasonal snowcover; a critical component of regional hydrology and ecosystem functions. The development of low-cost, open- source wireless sensor networks (WSN) however offers an opportunity to address data gaps in these regions and widen the availability of environmental monitoring to non-traditional audiences. Here we present a WSN, 'SnowBot' for snow-related applications built on the Arduino platform and utilizing Long Range Radio (LoRa) for two-way, local inter-node communication, and the Iridium constellation of satellites for data transmission from any global location with a clear view of the sky. The electronic hardware in each LoRa-enabled sensor node in our WSN costs approximately 270 USD, the addition of an Iridium modem in a ‘base-station’ configuration costs an additional ​250 USD and allows easy integration with Google Sheets, or other services, for near real-time data dissemination. As SnowBots are financially accessible, freely available to replicate, and have data streams that can be easily made publicly accessible, we envision their deployment in developing countries and/or in public-facing applications.

## Instructions for assembling a SnowBot

### Hardware assembly

The following section details how to assemble the hardware components of a SnowBot that works with the [example code](https://github.com/chrislcosgrove/snowBot/blob/main/code/snowBot_example/snowBot_example.ino). 

#### Microcontroller unit assembly
##### Assembling the [Adafruit Feather M0 LoRa](https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module) microcontroller
![](./img/feather_M0_LoRa_1.JPG)

Assembled [Adafruit Feather M0 LoRa](https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module) with stacking pins

![](./img/feather_M0_LoRa_2.JPG)

Side view with stacking header pins showing.

The first step to building the complete microcontroller unit is to solder [stacking headers](https://www.adafruit.com/product/2830) onto the [Adafruit Feather M0 LoRa](https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module). These headers allow the additional Adafruit Feather components to stack above the microcontroller, and for the microcontroller to fit into the terminal block. The headers line up with the outer row of pins on the microcontroller’s PCB - see images and instructions in the above links for further details and good suggestions on how to solder.

![](./img/feather_M0_LoRa_3.JPG)

Inverted microcontroller. [Arrow points to the uFL SMT Antenna Connector.](https://www.amazon.com/Adafruit-uFL-Antenna-Connector-ADA1661/dp/B06VVG9VM7)

Next is to solder on the [uFL SMT Antenna Connector](https://www.amazon.com/Adafruit-uFL-Antenna-Connector-ADA1661/dp/B06VVG9VM7) to the back side of the microcontroller. This is exceptionally fiddly and should be done when calm, well-rested, and of steady hand, ideally in bright light conditions. Tips for success are;
To find a comfortable working angle and secure the assembled microcontroller in that angle on your workstationUse the finest soldering iron tip you haveUse tiny bits of Blu Tack, or equivalent, to hold the antenna connector in place, leaving free the pad you are soldering firstBe as gentle and as patient as you can when applying the tip of the soldering iron to the soldering surface
Detailed instructions for the above, including an important note about the orientation of the antenna connector, can be found [here](https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module/antenna-options).

You may need a good few tries and connectors before you get a neat attachment - don’t stress about this and use a solder vacuum to remove failed attempts.
Lastly, label the microcontroller with a unique number, or letter/number combination. There’s two small site spaces on its underside that a fine, permanent marker can be used on.

##### Assembling the [Adafruit Feather Precision RTC](https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module) shield 

![](./img/feather_RTC_1.JPG)

Assembled [Adafruit Feather Precision RTC](https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module) with coin cell battery installed.

![](./img/feather_RTC_2.JPG)

Inverted assembled [Adafruit Feather Precision RTC](https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module) showing male pins.

As with the microcontroller above, the first step in assembling the [Adafruit Feather Precision RTC](https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module) (RTC = Real Time Clock) is to solder [stacking headers](https://www.adafruit.com/product/2830) into their corresponding pins (see images above). At this point you can also put in the [CR1220 coin cell battery](https://www.adafruit.com/product/380) into its holder.

![](./img/feather_RTC_3.JPG)

Assembled [Adafruit Feather Precision RTC](https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module). Arrows point to the location of a wire from pin 9 to the Int (interrupt pin)

The next step is to take a ~2 cm piece of jumper wire ([e.g. from this sort of kit](https://www.newark.com/global-specialties/wk-2/jumper-assortment-kit-140-pieces/dp/10R0134)) and solder it between the SQW/INT pin found in the centre of the PCB and pin 9 found on the shorter row of pins. This wire enables the RTC to ‘wake’ the microcontroller at preset intervals so that it can collect measurements, and store and transmit data. 

If you have the RTC with the button facing you, pin 9 is the 4th pin away from you on the right hand side - see above image. It’s best to solder so the bridge of the wire is on the top side of the RTC and its tips are underneath. To do this place the jumper wire in position and use a piece of Blu Tack or equivalent to hold it in place before flipping the RTC over and securing it to your workstation to solder.

##### Assembling the [Adafruit Feather Datalogger Shield](https://www.adafruit.com/product/2922)

![](./img/feather_datalogger_1.JPG)

Assembled [Adafruit Feather Datalogger Shield](https://www.adafruit.com/product/2922), note the inserted microSD card, short, non-stacking headers, and empty coin-cell battery holder

![](./img/feather_datalogger_3.JPG)

Side-view of the assembled [Adafruit Feather Datalogger Shield](https://www.adafruit.com/product/2922), note the clipped off male headers of the SCL/SDA pins

Similar to the microcontroller and RTC above, the first step in assembling the [Adafruit Feather Datalogger Shield](https://www.adafruit.com/product/2922) is to solder [headers](https://www.adafruit.com/product/2830) into their corresponding pins (see images above). However, the headers in this case are of the non-stacking, male variety that are included as standard with the shield. After lining up the headers in their corresponding pins, solder on the top side of the shield - see above images.

Once the soldering is complete the two headers that correspond to the SCL/SDA pins of the shield need to be clipped off - see above image. Be careful not clip off any further headers! These pins are on the shorter length of header, and those furthest from you if you have the coin-cell holder facing you and the SD card holder away from you - see above image.
At this point you can insert the microSd card into its slot - now is also a good time to label the SD card the same number, or letter/number combination, you gave the microcontroller.

You should be wondering why we don’t insert a coin cell battery into this shield, or indeed why we have two shields with RTCs. This is due to the Datalogger shield having an RTC that can’t be easily programmed to wake up / sleep the microcontroller at preset intervals - hence the use of the Precision RTC shield, which can handle these functions, but doesn’t have the SD card functionality. We clip the SCL/SDA pins so that the RTC on the Datalogger shield doesn’t interfere with the Precision RTC.

##### Completing the SnowBot microcontroller unit with a [Terminal Block Breakout](https://www.adafruit.com/product/2926)

![](./img/feather_assembled_unit.JPG)Complete microcontroller unit.

The microcontroller unit is assembled by stacking the Feather M0 LoRa Microcontroller on top of a [Terminal Block Breakout](https://www.adafruit.com/product/2926) first, then stacking the Precision RTC shield above the Microcontroller, and lastly the Datalogger shield atop the RTC - see above image. This order is important and take extra care to make sure all of the male headers line up properly with their female equivalents! The Terminal Block allows easy attachment of the various sensors used by the SnowBot, as well as the RockBlock.



#### Attaching sensors, RockBlock and antennas to the microcontroller unit. 

##### Attaching the [MaxBotix® MB7374 HRXL-MaxSonar-WRST7 Ultrasonic Precision Range Finder ](https://www.maxbotix.com/Ultrasonic_Sensors/MB7374.htm)

![](./img/maxbotix.JPG)

##### [MB7374 HRXL-MaxSonar-WRST7 Ultrasonic Precision Range Finder](https://www.maxbotix.com/Ultrasonic_Sensors/MB7374.htm)

MaxBotix® supply a wide range of ultrasonic sensors for use in a variety of environments and applications. For detecting the distance to the snow surface, which if you know the height of the sensor allows you to calculate the depth of the snow, the [MB7374 HRXL-MaxSonar-WRST7 Ultrasonic Precision Range Finder ](https://www.maxbotix.com/Ultrasonic_Sensors/MB7374.htm)is a good choice as it’s weather resistant, easily mounting using PVC piping, and outputs its data using analog, pulse width and TTL serial for easy reading by the Feather M0 microcontroller and other Arduino based boards. Its specs are high quality; a resolution of 1 mm, range of 500 to 5000 mm, relatively low current draw, and factory-calibration for detecting snow. It’s worth noting that other sensors exist in the range which have different filtering protocols and beam patterns (see datasheets).

To attach the MaxBotix to the microcontroller you need to first attach a suitable cable if you haven’t chosen to purchase a ready cabled sensor from the manufacture. A suitable cable needs to have a minimum of 4 strands and be ~22 AWG gauge - [this is a good example](https://www.homedepot.com/p/Southwire-By-the-Foot-22-4-Gray-Stranded-CU-CL3R-Shielded-Security-Cable-57572499/204725191). For this project I opted for Pulse Width output from the MaxBotix and hence soldered the 4 strands of my cable (see pic below and [sensor datasheet here](https://www.maxbotix.com/documents/HRXL-MaxSonar-WRS_Datasheet.pdf)) to the following pins on the sensor;

- red strand = V+ Pin 6 on MaxBotix sensor

- black strand = GND Pin 7 on MaxBotix sensor

- green strand = Pulse Width Output Pin 2 on MaxBotix sensor

- white strand = Ranging Start/Stop Pin 4 on MaxBotix sensor

After soldering the cable to the sensor ensure that you adequately weather-proof the connections.

Then to attach the sensor to the microcontroller unit the below wiring is used to the Terminal Breakout;

- red strand = 3v3 pin on Terminal Breakout (multiple choices)
- black strand = GND pin on Terminal Breakout (multiple choices)
- green strand = pin 11 on Terminal Breakout (7th pin up on right hand side if on/off switch faces you)
- white strand = pin 5 on Terminal Breakout (3rd pin up on right hand side if on/off switch faces you)

The red/black strands hence power the sensor, the green strand transmits the output of the sensor when it’s taken a measurement, and the white strand allows for the sensor to be turned on and off to save power. If you have purchased a pre-cabled sensor it’s just a case of connecting the appropriate strands (the colours will likely be different!).

Note that different pins can be used for reading the pulse width output and turning the sensor on and off - these pins just correspond to the example code uploaded here.

![](./img/maxbotix_wiring.JPG)

Wiring for the [MB7374 HRXL-MaxSonar-WRST7 Ultrasonic Precision Range Finder](https://www.maxbotix.com/Ultrasonic_Sensors/MB7374.htm) to the Terminal Block. 

##### Attaching the Temperature and Relative Humidity Sensor



##### Attaching the RockBlock Iridium Communicator



### Using the example SnowBot program

The [example SnowBot program](https://github.com/chrislcosgrove/snowBot/blob/main/code/snowBot_example/snowBot_example.ino) is field-tested and stable. It has built-in flexibility for different sampling, as well as LoRa and Iridium data transmission, frequencies and can support 10s of SnowBots in a ‘star’ type network topology. A star topology means that there is one central node, or ‘base-station’, that all the other nodes communicate with but there is no communication between non-base-station nodes. This is opposed to a mesh-type topology where all nodes are connected and data can be relayed throughout the network. At the time of writing a mesh version of the program is in development.

There are a great many good resources available online in basic Arduino programming – if this project is your first experience with the platform it is recommended that you review some tutorials for beginners - see https://www.arduino.cc/en/Tutorial/HomePage or https://www.makerspaces.com/arduino-uno-tutorial-beginners/ for example. On these pages, you will find instructions for how to load a program onto a microcontroller, install libraries, as well as basic guidance on the Arduino programming language. In these instructions, we assume that users have experience with these tasks and instead go through the details of the example SnowBot program.

##### Example program code structure

The code in the SnowBot program example is however structured in the following sections;

- Code header; title, date, author, hardware components needed, comments/description

- Constants; change to activate/deactivate sections of the program
- Libraries; list of libraries used in the program and where to source them online
- Pin definitions; microcontroller pins used to control specific hardware elements
- Object instantiations; code elements specific to particular libraries set-up
- Statistic objects; data storage elements specified using the Statistics library
- User-defined global variable declarations; specific variables that allow the SnowBot to easily programmed to sample at different frequencies, listen for different periods for new messages etc.

- Datagram set-up; specific object instantiation depending on whether the current SnowBot is a node or a base stationGlobal variables; declaration of zeroed, or empty, global variables for data storage and other functionality
- Structure to store and send data; a data-structure where measured variables can be compressed into byte formatFunction prototypes; list of functions included in the program
- Set-up; code that initiates each time the microcontroller is started
- Loop; code that loops following the set-up
- Functions; supporting functions for code in the set-up and loop

What follows are more detailed explanations of the most pertinent sections including commentary on design decisions and areas with scope to develop further.

##### Constants

```c++
/****************************
Constants
****************************/
#define NODE_STATION   false    // Set if node is just to transmit messages by LoRa
#define BASE_STATION   true     // Set if RockBlock is installed and you wish to listen for LoRa messages
#define DEBUG          false    // Output debug messages to Serial Monitor
#define DEPLOY         true     // Disable debugging messages for deployment
#define DIAGNOSTICS    false    // Output Iridium diagnostic messages to Serial Monitor
#define ROCKBLOCK      true     // If base station define whether RockBlock is used
```

The constants defined at the start of the program allow you to easily swap, using the boolean argument `true` or `false`, between defining the SnowBot as a 'Node' `#define NODE_STATION ` , or as a 'Base Station' `#define BASE_STATION` (see SnowBot overview above), setting up the SnowBot in a debug mode `#define DEBUG`, in deployment mode `#define DEPLOY`, print RockBlock diagnostics to the serial monitor `#define DIAGNOSTICS`, and specify whether a RockBlock is being used `#define ROCKBLOCK`.

Throughout the program you can identify where these constants operate on specific code blocks by searching for `#if` and `#endif` sections followed by the constant name. For example, in the below code block if the `NODE_STATION` constant is set to `true` the program will set-up the LoRa radio datagram manager so that the SnowBot sends messages to be received, rather than listens for messages, as in the case of the base station.

```c++
#if NODE_STATION
RHReliableDatagram manager(rf95, NODE_ADDRESS);
#endif
```

The `DEBUG` constant both sets the first alarm to wake the SnowBot at the turn of the next minute, instead of at the turn of the hour, and allows Serial printing of various other information. The `BASE_STATION` constant sets up the SnowBot to receive LoRa messages from other nodes. The `ROCKBLOCK` function, when used in conjunction with the `BASE_STATION` function enables RockBlock operation. **Note that the program will not work correctly if** `BASE_STATION` **or** `NODE_STATION` **are both defined as** `true` **or** `false`**, likewise** `DEBUG` **and** `DEPLOY`**.** We recommend reviewing the code blocks corresponding to each constant definition to get a deeper sense of how they operate.

##### Libraries

```c++
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
```

The above libraries need to be installed by either using Arduino's Library Manager, adding a zip file, manually, or otherwise - see [here](https://www.arduino.cc/en/guide/libraries) for more details. 

##### Pin definitions

```c++
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
```

The above pin definitions are correct and tested for the hardware outlined in the  header of the SnowBot example program and BOM, and assembled per the instructions above. If you are using a different development board, or sensors, you will likely need to change these according to your set-up.

##### Object Instantiations

```c++
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
```

These objects are instantiated to carry out important functions relating to reading the sensors, storing data, handling time, and transmitting data via LoRa and RockBlock. If you are using the same hardware as outlined in the BOM and header of the example program you should not need to change these. However, according to geographical location you will need to change the LoRa frequency - see [here](https://en.wikipedia.org/wiki/LoRa). The SnowBot example program is set up to operate in North America with a LoRa frequency of 915 MHz - this line `#define RF95_FREQ 915.0 ` sets the desired frequency in MHz.

##### Statistic Objects

```c++
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
```

The Statistic library allows creation of objects that can store arrays of data, for example sensor measurements, that you can then run simple statistical tests on. In the SnowBot example program they're used to hold sensor measurements taken at intervals, e.g. 5 minutes, that can then be averaged or queried for their minimum, maximum, standard deviation, or count, over a specified period, e.g. 1 hour. This gives some flexibility in sampling and data transmission strategy, as described in more detail below. To best understand how the Statistic objects work in the program, search for each of them, e.g. hit Ctrl+F and type in `humidityStats`, and follow how and when data are added to the object, what and when statistical queries are made of the objects, and when data is cleared from the object.

**Note on MaxBotix measurements.** Ultrasonic distance sensors are well-known to produce noisy data when measuring the distance to the snow surface, especially in windy or otherwise disturbed conditions. Hence, for the purposes of improving quality control of the MaxBotix distance readings, statistic objects are created to contain the average, standard deviation, maximum and minimum distance measured at each sampling interval. The function to operate and read the MaxBotix at each sampling interval, allows for 30 distance measurements over ~5 seconds to reduce the effect of erroneous measurements. The average, standard deviation, maximum and minimum distance from these 30 readings is then found from a local statistics object created within the function and stored in their corresponding global statistic objects (listed above).  Useful indicators of measurement quality can then be derived from data averaged over a longer period, e.g. an hour (see section on User defined global variable declarations below for more detail on sample intervals and average/transmission intervals). Additionally, the MaxBotix function also filters out any measurements outside of the working range of the instrument (550 to 4950 mm), labelling them as a NaN. The count of NaNs at each measurement interval is kept in a further statistics object `MaxbotixStats_nan` and is a useful indicator of measurement stability - fewer NaNs and a lower standard deviation indicate a higher quality measurement. We encourage users to explore each of the sensor reading functions to better understand how the program operates.



##### User defined global variables

```c++
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
```

The code section 'User defined global variables' is the most important in the [SnowBot example program](https://github.com/chrislcosgrove/snowBot/blob/main/code/snowBot_example/snowBot_example.ino). Here you set the identification of the SnowBot and specify its data sampling and data transmission strategy. What follows is a more detailed explanation of the variables and how they function.

- `node_name` - this is the SnowBot's identification number in character format, i.e. specified within quotes like `"<name>"`. This will be used to name the .csv files storing the data on the microSD card. In the example program, `"s03"` is used to indicate 'SnowBot #3'.
- `node_number` - this is the number of the SnowBot and is always in an integer format. It is used in the program for identification purposes as the first item in the row of data written to the SD card at each sampling interval. It is also the first item in the data transmission structure, which will be decoded and written on the base station's SD card if the current SnowBot is just a normal sampling node. It is also used as the address for the node in the Reliable Datagram manager to transmit data via LoRa so hence must be unique! The Reliable Datagram, in theory, can handle 256 seperate SnowBot nodes, though this would need to be tested extensively for reliability before deployment.
- `base_station_number` - this is the number of the SnowBot which will act as the base station, collecting sampling nodes' data and sending it via Iridium. This number **must** be consistent across the network of SnowBots as it serves as the address in the Reliable Datagram manager for LoRa transmissions. If the current SnowBot is the base station keep tis the same as the `node_number`.
- `total_nodes` - this is the total number of SnowBots deployed in the network and functions alongside the chosen `transmitInterval` to ensure that messages are sent before the RockBlock's 340 byte limit for 1 message is reached, see below for more details.

