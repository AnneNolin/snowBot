## Instructions for assembling a SnowBot
Author - Chris Cosgrove

### SnowBot overview

### Hardware assembly
#### Microcontroller unit assembly
##### Assembling the [Adafruit Feather M0 LoRa](https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module) microcontroller
![](/Users/chris/Desktop/snowBot/img/feather_M0_LoRa_1.JPG)

Assembled [Adafruit Feather M0 LoRa](https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module) with stacking pins

![](/Users/chris/Desktop/snowBot/img/feather_M0_LoRa_2.JPG)

Side view with stacking header pins showing.

The first step to building the complete microcontroller unit is to solder [stacking headers](https://www.adafruit.com/product/2830) onto the [Adafruit Feather M0 LoRa](https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module). These headers allow the additional Adafruit Feather components to stack above the microcontroller, and for the microcontroller to fit into the terminal block. The headers line up with the outer row of pins on the microcontroller’s PCB - see images and instructions in the above links for further details and good suggestions on how to solder.

![](/Users/chris/Desktop/snowBot/img/feather_M0_LoRa_3.JPG)

Inverted microcontroller. [Arrow points to the uFL SMT Antenna Connector.](https://www.amazon.com/Adafruit-uFL-Antenna-Connector-ADA1661/dp/B06VVG9VM7)

Next is to solder on the [uFL SMT Antenna Connector](https://www.amazon.com/Adafruit-uFL-Antenna-Connector-ADA1661/dp/B06VVG9VM7) to the back side of the microcontroller. This is exceptionally fiddly and should be done when calm, well-rested, and of steady hand, ideally in bright light conditions. Tips for success are;
To find a comfortable working angle and secure the assembled microcontroller in that angle on your workstationUse the finest soldering iron tip you haveUse tiny bits of Blu Tack, or equivalent, to hold the antenna connector in place, leaving free the pad you are soldering firstBe as gentle and as patient as you can when applying the tip of the soldering iron to the soldering surface
Detailed instructions for the above, including an important note about the orientation of the antenna connector, can be found [here](https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module/antenna-options).

You may need a good few tries and connectors before you get a neat attachment - don’t stress about this and use a solder vacuum to remove failed attempts.
Lastly, label the microcontroller with a unique number, or letter/number combination. There’s two small site spaces on its underside that a fine, permanent marker can be used on.

##### Assembling the [Adafruit Feather Precision RTC](https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module) shield 

![](/Users/chris/Desktop/snowBot/img/feather_RTC_1.JPG)

Assembled [Adafruit Feather Precision RTC](https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module) with coin cell battery installed.

![](/Users/chris/Desktop/snowBot/img/feather_RTC_2.JPG)

Inverted assembled [Adafruit Feather Precision RTC](https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module) showing male pins.

As with the microcontroller above, the first step in assembling the [Adafruit Feather Precision RTC](https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module) (RTC = Real Time Clock) is to solder [stacking headers](https://www.adafruit.com/product/2830) into their corresponding pins (see images above). At this point you can also put in the [CR1220 coin cell battery](https://www.adafruit.com/product/380) into its holder.

![](/Users/chris/Desktop/snowBot/img/feather_RTC_3.JPG)

Assembled [Adafruit Feather Precision RTC](https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module). Arrows point to the location of a wire from pin 9 to the Int (interrupt pin)

The next step is to take a ~2 cm piece of jumper wire ([e.g. from this sort of kit](https://www.newark.com/global-specialties/wk-2/jumper-assortment-kit-140-pieces/dp/10R0134)) and solder it between the SQW/INT pin found in the centre of the PCB and pin 9 found on the shorter row of pins. This wire enables the RTC to ‘wake’ the microcontroller at preset intervals so that it can collect measurements, and store and transmit data. 

If you have the RTC with the button facing you, pin 9 is the 4th pin away from you on the right hand side - see above image. It’s best to solder so the bridge of the wire is on the top side of the RTC and its tips are underneath. To do this place the jumper wire in position and use a piece of Blu Tack or equivalent to hold it in place before flipping the RTC over and securing it to your workstation to solder.

##### Assembling the [Adafruit Feather Datalogger Shield](https://www.adafruit.com/product/2922)

![](/Users/chris/Desktop/snowBot/img/feather_datalogger_1.JPG)

Assembled [Adafruit Feather Datalogger Shield](https://www.adafruit.com/product/2922), note the inserted microSD card, short, non-stacking headers, and empty coin-cell battery holder

![](/Users/chris/Desktop/snowBot/img/feather_datalogger_3.JPG)

Side-view of the assembled [Adafruit Feather Datalogger Shield](https://www.adafruit.com/product/2922), note the clipped off male headers of the SCL/SDA pins

Similar to the microcontroller and RTC above, the first step in assembling the [Adafruit Feather Datalogger Shield](https://www.adafruit.com/product/2922) is to solder [headers](https://www.adafruit.com/product/2830) into their corresponding pins (see images above). However, the headers in this case are of the non-stacking, male variety that are included as standard with the shield. After lining up the headers in their corresponding pins, solder on the top side of the shield - see above images.

Once the soldering is complete the two headers that correspond to the SCL/SDA pins of the shield need to be clipped off - see above image. Be careful not clip off any further headers! These pins are on the shorter length of header, and those furthest from you if you have the coin-cell holder facing you and the SD card holder away from you - see above image.
At this point you can insert the microSd card into its slot - now is also a good time to label the SD card the same number, or letter/number combination, you gave the microcontroller.

You should be wondering why we don’t insert a coin cell battery into this shield, or indeed why we have two shields with RTCs. This is due to the Datalogger shield having an RTC that can’t be easily programmed to wake up / sleep the microcontroller at preset intervals - hence the use of the Precision RTC shield, which can handle these functions, but doesn’t have the SD card functionality. We clip the SCL/SDA pins so that the RTC on the Datalogger shield doesn’t interfere with the Precision RTC.

##### Completing the SnowBot microcontroller unit with a [Terminal Block Breakout](https://www.adafruit.com/product/2926)

![](/Users/chris/Desktop/snowBot/img/feather_assembled_unit.JPG)Complete microcontroller unit.

The microcontroller unit is assembled by stacking the Feather M0 LoRa Microcontroller on top of a [Terminal Block Breakout](https://www.adafruit.com/product/2926) first, then stacking the Precision RTC shield above the Microcontroller, and lastly the Datalogger shield atop the RTC - see above image. This order is important and take extra care to make sure all of the male headers line up properly with their female equivalents! The Terminal Block allows easy attachment of the various sensors used by the SnowBot, as well as the RockBlock.



#### Attaching sensors, RockBlock and antennas to the microcontroller unit. 

##### Attaching the [MaxBotix® MB7374 HRXL-MaxSonar-WRST7 Ultrasonic Precision Range Finder ](https://www.maxbotix.com/Ultrasonic_Sensors/MB7374.htm)

![](/Users/chris/Desktop/snowBot/img/maxbotix.JPG)

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

![](/Users/chris/Desktop/snowBot/img/maxbotix_wiring.JPG)

Wiring for the [MB7374 HRXL-MaxSonar-WRST7 Ultrasonic Precision Range Finder](https://www.maxbotix.com/Ultrasonic_Sensors/MB7374.htm) to the Terminal Block. 

##### Attaching the Temperature and Relative Humidity Sensor



##### Attaching the RockBlock Iridium Communicator



### Using the example SnowBot program

The example SnowBot program (insert a link to Github - working off sb03_02Feb21_1hrSample_1hrLoRa_6hrRckBlk.ino) is field-tested and stable. It has built-in flexibility for different sampling, as well as LoRa and Iridium data transmission, frequencies and can support 10s of SnowBots in a ‘star’ type network topology. A star topology means that there is one central node, or ‘base-station’, that all the other nodes communicate with but there is no communication between non-base-station nodes. This is opposed to a mesh-type topology where all nodes are connected and data can be relayed throughout the network. At the time of writing a mesh version of the program is in development.

There are a great many good resources available online in basic Arduino programming – if this project is your first experience with the platform it is recommended that you review some tutorials for beginners - see https://www.arduino.cc/en/Tutorial/HomePage or https://www.makerspaces.com/arduino-uno-tutorial-beginners/ for example. On these pages, you will find instructions for how to load a program onto a microcontroller, install libraries, as well as basic guidance on the Arduino programming language. In these instructions, we assume that users have experience with these tasks and instead go through the details of the example SnowBot program

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
- Set-up; code that initiates each time the microcontroller is startedLoop; code that loops following the set-up
- Functions; supporting functions for code in the set-up and loop

What follows are more detailed explanations of the most pertinent sections including commentary on design decisions and areas with scope to develop further.

##### Constants

