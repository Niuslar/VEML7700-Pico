# VEML7700 Library for Raspberry Pi Pico / Pico W

This is a simple C++ library to interact with the VEML7700 ambient light sensor (ALS).

The sensor can read between 0lx and 120klx with varying resolution depending on the configuration. For more information please refer to the datasheet. 

# Using this library
To use this library, clone this repository into your project add it to your main CMakeLists.txt file with 
`target_link_libraries(target veml7700)`

# Example 

```
/* Simple Application to read lux levels every 1 second */
#include "VEML770.h"
#include "pico/stdlib.h"
#include <stdio.h>

int main()
{
    stdio_init_all();

    VEML7700 LightSensor;

    // Use GPIO pins 26 and 27 for I2C (which is I2C1)
    LightSensor.Init(26,27,i2c1);
    
    while (1)
    {
        // Read lux levels every 1s
        float lightLevel = LightSensor.ReadLux();
        printf("Ambient light = %.2f lux\n", lightLevel);
        sleep_ms(1000);
    }
}
```
