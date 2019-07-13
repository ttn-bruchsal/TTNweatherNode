# TTNweatherNode
Simple low power Arduino Pro Mini weather node based on the BME280 sensor using a RFM95W Lora Module

The node and this code was used for demonstration purpose by 
TheThingsNetwork Community Bruchsal in the course of an entry level
workshop.

## Libraries
### CayenneLPP by Electronic Cats 1.0.1
### SparkFun BME280 by SparkFun Electronics 2.0.4
### Low Power Library by Rocketscream
https://github.com/rocketscream/Low-Power

### MCCI LoRaWAN LMIC library by IBM, Matthis Kooijman ... 2.3.2

Adjust MCCI_LoRaWAN_LMIC_library/project_config/lmic_project_config.h
Example File included.

```
// project-specific definitions
#define CFG_eu868 1
//#define CFG_us915 1
//#define CFG_au921 1
//#define CFG_as923 1
// #define LMIC_COUNTRY_CODE LMIC_COUNTRY_CODE_JP       /* for as923-JP */
//#define CFG_in866 1
#define CFG_sx1276_radio 1
#define LMIC_USE_INTERRUPTS
#define DISABLE_PING
#define DISABLE_BEACONS
#define DISABLE_JOIN
//#define USE_ORIGINAL_AES
```
