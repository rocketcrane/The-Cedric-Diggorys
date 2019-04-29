#Adafruit TCS34725 Color Sensor Driver with software SDA / SCL emulation#

This driver is for the Adafruit TCS34725 Breakout.
    ------> http://www.adafruit.com/products/1334
	
	

## About this Driver ##

This is fork for original TCS34725 driver.
You can read about it here: https://github.com/adafruit/Adafruit_TCS34725

## How to use ##

1. Install SoftwareWire library with my modification with empty constructor --> https://github.com/Fire7/SoftwareWire
2. Use it with new constructor:
```
#include "Adafruit_TCS34725softi2c.h"

// You can use any digital pin for emulate SDA / SCL
#define SDApin 8
#define SCLpin 9

/* Initialise with specific int time and gain values and custom SDA / SCL pin */
Adafruit_TCS34725softi2c tcs = Adafruit_TCS34725softi2c(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X, SDApin, SCLpin);

```

