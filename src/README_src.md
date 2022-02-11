Disclaimer:
--------------
THE SOFTWARE IS RELEASED INTO THE PUBLIC DOMAIN.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, NONINFRINGEMENT,
SECURITY, SATISFACTORY QUALITY, AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL EPSON BE LIABLE FOR ANY LOSS, DAMAGE OR CLAIM, ARISING FROM OR
IN CONNECTION WITH THE SOFTWARE OR THE USE OF THE SOFTWARE.


Test machine:
--------------
  For using IMU SPI interface, this is only supported on Raspberry Pi using wiringPi Library 
      RPi with Ubuntu Mate-18.04.4-desktop-armhf
      [Unofficial wiringPi](https://github.com/WiringPi/WiringPi/)

NOTE: This document only applies to compiling and running this C software independently on RaspberryPi with embedded Linux.
      For proper operation in ROS environment refer to the README.md found elsewhere in the root of the archive.


Important for RPi SPI Interface:
-----------------------------
1. After the installation process, apply any updates.
   sudo apt-get update
   sudo apt-get upgrade
   
2. Use raspi-config utility to enable the SPI interface:
   sudo raspi-config
   
   Choose the advanced Options -> SPI -> enable SPI kernel module to be loaded by default "Yes"
   sudo reboot
   
3. Verify the SPI interface is enabled.
   lsmod | grep spi_
   ls /dev | grep spi

4. Install the wiringPi library.
   This should already be installed if running a RPi-specific Linux distro, otherwise go to [Unofficial wiringPi](https://github.com/WiringPi/WiringPi/)

5. The application is designed with the following pinmapping:
    Epson IMU                   Raspberry Pi
    ---------------------------------------------------
    EPSON_RESET                 RPI_GPIO_P1_15 (GPIO22) 
    EPSON_DRDY                  RPI_GPIO_P1_18 (GPIO24)
    EPSON_CS                    RPI_GPIO_P1_16 (GPIO23)
    SPI_SCK                     RPI_GPIO_P1_23 (GPIO11)
    SPI_MISO                    RPI_GPIO_P1_21 (GPIO9)
    SPI_MOSI                    RPI_GPIO_P1_19 (GPIO10)


Important for usleep():
-----------------------
NOTE: In the hcl_rpi.c, there are functions for time delays in millisecond and microseconds using seDelayMS() and seDelayMicroSecs(), respectively.

On embedded Linux platforms, these may need to be redirected to HW specific delay routines if usleep() is not supported.

For example on RaspberryPi, the time delay functions for millisecond and microseconds can be redirected to WiringPi library delay() and delayMicroseconds(), respectively.


Important for GPIO Usage:
-------------------------
Because this driver connects to the IMU using the SPI interface, the use of GPIO pins for connecting to the CS# & DRDY is MANDATORY.

When possible, connecting the RESET# is recommended to force Hardware Reset during every IMU initialization for better robustness.

This code will make use of GPIO functions and is structured to easily redirect to low-level hardware GPIO function calls for ease of implementation.
   
There is no standard method to implement GPIO connections on embedded Linux platform, but the following files typically need changes:

  - src/hcl_<platform>.c
  - src/hcl_gpio_<platform>.c
  - src/hcl_gpio.h

Typically, an external library needs to be invoked to initialize the library & GPIO HW functions.

This requires changes to hcl_<platform>.c

  - add #include to external library near the top of hcl_<platform>.c
  - add #include hcl_gpio.h near the top of the hcl_<platform>.c
  - add the initialization call inside the seInit() function in hcl_<platform>.c
    
For example on an Raspberry Pi, changes to hcl_rpi.c:

```
  .
  .
  .
  #include <stdint.h>
  #include <stdio.h>
  #include <wiringPi.h>  // <== Added external library
  
  int seInit(void)
  {
    // Initialize wiringPi libraries                                                   // <== Added 
    printf("\r\nInitializing libraries...");                                           // <== Added
    if(wiringPiSetupGpio() != 0) {                                                     // <== Added external library initialization
      printf("\r\nError: could not initialize wiringPI libraries. Exiting...\r\n");    // <== Added
      return NG;                                                                       // <== Added
    }                                                                                  // <== Added
    printf("...done.");
  
    return OK;
  }
  .
  .
  .

```

Typically, the GPIO pins need to be assigned according to pin numbering specific to the HW platform.

This requires changes to hcl_gpio.h   

For example on an Raspberry Pi, changes to hcl_gpio.h with the following pinmapping:

```
    Epson IMU                   Raspberry Pi
    ---------------------------------------------------
    EPSON_RESET                 RPI_GPIO_P1_15 (GPIO22) Output
	EPSON_CS                    RPI_GPIO_P1_16 (GPIO23) Output
    EPSON_DRDY                  RPI_GPIO_P1_18 (GPIO24) Input

```

```   
  // Prototypes for generic GPIO functions
  int gpioInit(void);
  int gpioRelease(void);
  
  void gpioSet(uint8_t pin);
  void gpioClr(uint8_t pin);
  uint8_t gpioGetPinLevel(uint8_t pin);
  
  #define RPI_GPIO_P1_15              22                    // <== Added 
  #define RPI_GPIO_P1_16              23                    // <== Added
  #define RPI_GPIO_P1_18              24                    // <== Added 
  
  #define EPSON_RESET                 RPI_GPIO_P1_15        // <== Added 
  #define EPSON_CS                    RPI_GPIO_P1_16        // <== Added
  #define EPSON_DRDY                  RPI_GPIO_P1_18        // <== Added 
  .
  . 
  . 
```


Typically, the external library will have GPIO pin control such as set_output, set_input, set, reset, read_pin_level, etc...
  
This requires changes to hcl_gpio_<platform>.c
  
  - Redirect function calls in hcl_gpio.c for gpioInit(), gpioRelease(), gpioSet(), gpioClr(), gpioGetPinLevel() to equivalent external library pin control functions.

  - For example on an Raspberry Pi, changes to hcl_gpio_rpi.c:

```   
  #include "hcl.h"
  #include "hcl_gpio.h"
  #include <wiringPi.h>                         // <== Added external library
  .
  .
  .

  int gpioInit(void)
  {
    pinMode(EPSON_RESET, OUTPUT);               // <== Added external call RESET Output Pin
    pinMode(EPSON_CS, OUTPUT);                  // <== Added external call RESET Output Pin
    pinMode(EPSON_DRDY, INPUT);                 // <== Added external call DRDY Input Pin
    pullUpDnControl(EPSON_DRDY, PUD_OFF) ;      // <== Added external call Disable any internal pullup or pulldown resistance

    return SUCCESS;
  }
  .
  .
  .
  int gpioRelease(void)
  {
    return SUCCESS;
  }
  .
  .
  .
  void gpioSet(uint8_t pin)
  {
    digitalWrite(pin, HIGH);                    // <== Added external call set pin HIGH 
  }
  
  .
  .
  .
  void gpioClr(uint8_t pin)
  {
    digitalWrite(pin, LOW);                     // <== Added external call set pin LOW
  }

  .
  .
  .
  uint8_t gpioGetPinLevel(uint8_t pin)
  {
    return (digitalRead(pin));                  // <== Added external call to return pin state of input pin
  }

  .
  . 
  . 
```

Important for Compiling:
-----------------------------
Compile the application:
    A. Run "make" specifying the <target> with "MODEL=xxx" parameters.
       Supported <target> options are: 
            screen or 
            csvlogger or 
            regdump or
            all
       Supported "MODEL=" parameters are:
            G354
            G364PDC0
            G364PDCA
            G365PDC0
            G365PDF0
            G370PDF0
            G325PDF1
            G365PDF1
            G365PDC1
            G370PDF1
            G325PDF0
            G320
            V340

       If "MODEL=" is not specified then it assumes MODEL=G365PDF0

       For example:
            make clean  <-- recommended before creating new builds
			or
			make screen MODEL=G364PDCA
            or
            make csvlogger MODEL=V340
            or
            make regdump MODEL=G365PDF0
            or
            make all MODEL=G370PDF1
    B. The executable will be in the found in the same folder as the Makefile and source files.
    
NOTE: Modify the EpsonOptions struct to configure sensor configuration settings in main() function in main_xxxx.c


How to run the program:
-----------------------
1. Run the executable from console (may require root access to execute if regular user can not access SPI device)
```
   sudo ./<executable filename>
```

2. The default csvlogger program creates CSV log of sensor data in a processed scaled log file for 1000 samples:
- GPIO2 = External counter reset active HIGH
- DRDY is enabled active HIGH
- Output date rate = 125 Hz
- Filter Tap = Moving Average TAP32
- Sensor Output:
  - 16-bit Gyro X,Y,Z Accel X,Y,Z ResetCounter Checksum

File Listing:
--------------
```
epson_imu_spi_driver_node.cpp   - ROS Driver C++ to C wrapper
                                - The ROS driver is built using ROS catkin_make environment

hcl.h                       - Dummy abstraction layer header which defines delay() functions
hcl_rpi.c                   - Abstraction layer for Raspberry Pi Platform
hcl_gpio_rpi.c              - Abstraction layer for GPIO control connection to RESET, DRDY, SCS# 
                              For Raspberry Pi assignment of pins RESET, DRDY, SCS#
                              Modify or replace if GPIO pins are to be used
hcl_gpio.h                  - Header for GPIO abstraction
hcl_spi_rpi.c               - Abstraction layer specific for SPI IF on Raspberry Pi which uses WiringPi library calls
hcl_spi.h                   - Header for SPI IF abstraction
main_csvlogger.c            - Test application - Initialize IMU, and read sensor data to CSV log file
main_regdump.c              - Test application - Output register settings to console for debug purpose
main_screen.c               - Test application - Initialize IMU, and read sensor data to console
main_helper.c               - Helper functions for console utilities
main_helper.h               - Header for helper functions for console utilities
Makefile                    - For make utility to compile test applications
README_src.md               - This file.
sensor_epsonCommon.c        - Common functions for Epson IMU
sensor_epsonCommon.h        - Header for common C functions of Epson IMU
sensor_epsonG320.c          - Model specific functions for Epson M-G320
sensor_epsonG320.h          - Model specific header for Epson M-G320
sensor_epsonG354.c          - Model specific functions for Epson M-G354
sensor_epsonG354.h          - Model specific header for Epson M-G354
sensor_epsonG364.c          - Model specific functions for Epson M-G364
sensor_epsonG364PDC0.h      - Model specific header for Epson M-G364PDC0
sensor_epsonG364PDCA.h      - Model specific header for Epson M-G364PDCA
sensor_epsonV340.c          - Model specific functions for Epson M-G340
sensor_epsonG340.h          - Model specific header for Epson M-V340
sensor_epsonG365.c          - Model specific functions for Epson M-G365
sensor_epsonG365PDC0.h      - Model specific header for Epson M-G365PDC0
sensor_epsonG365PDF0.h      - Model specific header for Epson M-G365PDF0
sensor_epsonG365PDC1.h      - Model specific header for Epson M-G365PDC1
sensor_epsonG365PDF1.h      - Model specific header for Epson M-G365PDF1
sensor_epsonG370.c          - Model specific functions for Epson M-G370
sensor_epsonG370PDF0.h      - Model specific header for Epson M-G370PDF0
sensor_epsonG370PDF1.h      - Model specific header for Epson M-G370PDF1
sensor_epsonG325.c          - Model specific functions for Epson M-G325
sensor_epsonG325PDF0.h      - Model specific header for Epson M-G325PDF0
sensor_epsonG325PDF1.h      - Model specific header for Epson M-G325PDF1
sensor_epsonV340.c          - Model specific functions for Epson M-G325
sensor_epsonV340.h          - Model specific header for Epson M-G325PDF0
sensor_epsonSpi.c           - SPI specific functions
```

Change Record:
--------------
```
2017-03-29  v1.0    - Initial SPI release
2018-04-04  v1.1    - Code cleanup, no functional changes
2018-11-30  v1.2    - Added support for G365/G370
2019-07-19  v1.3    - Added G325 support, Refactored code to be similar to latest UART version and use struct to initialize IMU
                    - Modified to be used as wrapper for ROS
                    - Modified to use wiringPi library for SPI/GPIO interface instead of BCM2835 Library
2020-03-06  v1.4    - Fixed bug in quaternion / euler output for epson_imu_driver_node.cpp
2020-03-24  v1.5    - Clean up README.md and minor refactor (no functional change)
2020-10-01  v1.6    - Added support for G325PDF1, G365PDC1, G365PDF1, G370PDF1 and bugfixes

```
