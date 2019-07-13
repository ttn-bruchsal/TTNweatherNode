/************************************************************************
This is a slightly modified version of Thomas Telkamp, Matthijs Kooijman 
and Terry Moore's excellent "ttn-abp" example code.

For licensing details regarding this code please see the accompanied LICENSE file.

The code needs the following great libraries to run, which can be installed
directly from the Arduino IDE, or, in case of the Low-Power library, from .zip:
https://github.com/sabas1080/CayenneLPP
https://github.com/mcci-catena/arduino-lmic
https://github.com/sparkfun/SparkFun_BME280_Arduino_Library
https://github.com/rocketscream/Low-Power
Please refer to their respective Licenses, too.

This code uses snipplets of example code from the included library projects.

For reference reasons this is the original copyright notice:
*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any resltriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses ABP (Activation-by-personalisation), where a DevAddr and
 * Session keys are preconfigured (unlike OTAA, where a DevEUI and
 * application key is configured, while the DevAddr and session keys are
 * assigned/generated in the over-the-air-activation procedure).
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!
 *
 * To use this sketch, first register your application and device with
 * the things network, to set or generate a DevAddr, NwkSKey and
 * AppSKey. Each device should have their own unique values for these
 * fields.
 *
 * Do not forget to define the radio type correctly in
 * arduino-lmic/project_config/lmic_project_config.h or from your BOARDS.txt.
 *
 *******************************************************************************/

 // References:
 // [feather] adafruit-feather-m0-radio-with-lora-module.pdf

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "SparkFunBME280.h"
#include "LowPower.h"

// If defined, the code will generate Cayenne compatible payload.
// Otherwise the sensor values will be coded as 16bit integers, each.
#define CAYENNE

// To get debug information on the serial console, uncomment the following
// 2 linese and comment the other 2 linese below.

//#define Sprintln(a) (Serial.println(a))
//#define Sprint(a) (Serial.print(a))
#define Sprintln(a)
#define Sprint(a)


#ifdef CAYENNE
#include <CayenneLPP.h>

CayenneLPP lpp(51);

#else
static uint8_t mydata[6];

#endif // CAYENNE


BME280 mySensor;


//
// For normal use, we require that you edit the sketch to replace FILLMEIN
// with values assigned by the TTN console. However, for regression tests,
// we want to be able to compile these scripts. The regression tests define
// COMPILE_REGRESSION_TEST, and in that case we define FILLMEIN to a non-
// working but innocuous value.
//
#ifdef COMPILE_REGRESSION_TEST
# define FILLMEIN 0
#else
# warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
# define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif

// LoRaWAN NwkSKey, network session key
static const PROGMEM u1_t NWKSKEY[16] = FILLMEIN ;

// LoRaWAN AppSKey, application session key
static const u1_t PROGMEM APPSKEY[16] = FILLMEIN ;

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed.
static const u4_t DEVADDR = FILLMEIN ; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in arduino-lmic/project_config/lmic_project_config.h,
// otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }


static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping
// Adapted for Feather M0 per p.10 of [feather]
const lmic_pinmap lmic_pins = {
    .nss = 10,                       // chip select on feather (rf95module) CS
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 5,                       // reset pin
    .dio = {2, 3, 4}, // assumes external jumpers [feather_lora_jumper]
                                    // DIO1 is on JP1-1: is io1 - we connect to GPO6
                                    // DIO1 is on JP5-3: is D2 - we connect to GPO5
};

extern volatile unsigned long timer0_millis;
void addMillis(unsigned long extra_millis) {
  uint8_t oldSREG = SREG;
  cli();
  timer0_millis += extra_millis;
  SREG = oldSREG;
  sei();
}

void do_sleep(unsigned int sleepyTime) {
  unsigned int eights = sleepyTime / 8;
  unsigned int fours = (sleepyTime % 8) / 4;
  unsigned int twos = ((sleepyTime % 8) % 4) / 2;
  unsigned int ones = ((sleepyTime % 8) % 4) % 2;

  for ( int x = 0; x < eights; x++) {
    // put the processor to sleep for 8 seconds
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  }
  for ( int x = 0; x < fours; x++) {
    // put the processor to sleep for 4 seconds
    LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);
  }
  for ( int x = 0; x < twos; x++) {
    // put the processor to sleep for 2 seconds
    LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
  }
  for ( int x = 0; x < ones; x++) {
    // put the processor to sleep for 1 seconds
    LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
  }
  
  addMillis(sleepyTime * 1000);
}

void onEvent (ev_t ev) {
#ifdef SERIALDEBUG        
    Sprint(os_getTime());
    Sprint(": ");
#endif    
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Sprintln(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Sprintln(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Sprintln(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Sprintln(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Sprintln(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Sprintln(F("EV_JOINED"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Sprintln(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Sprintln(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Sprintln(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Sprintln(F("Received ack"));
            if (LMIC.dataLen) {
              Sprintln(F("Received "));
              Sprintln(LMIC.dataLen);
              Sprintln(F(" bytes of payload"));
            }
            // Schedule next transmission
            do_sleep(TX_INTERVAL);
            
            //os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            os_setTimedCallback(&sendjob,os_getTime(), do_send);
            break;
        case EV_LOST_TSYNC:
            Sprintln(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Sprintln(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Sprintln(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Sprintln(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Sprintln(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Sprintln(F("EV_TXSTART"));
            break;
        default:
            Sprint(F("Unknown event: "));
            Sprintln((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Sprintln(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        mySensor.setMode(1); // One Reading
        delay(10);
        Sprint(F("Packet: "));
        
        Sprint(F("T: "));
        Sprint(mySensor.readTempC());
        Sprint(F("°C "));

        Sprint(F("H: "));
        Sprint(mySensor.readFloatHumidity());
        Sprint(F("%r.H. "));

        Sprint(F("P: "));
        Sprint(mySensor.readFloatPressure()/100); // hPa...
        Sprintln(F("hPa "));
  
#ifdef CAYENNE        
        lpp.reset();
        lpp.addTemperature(1, mySensor.readTempC());
        lpp.addRelativeHumidity(2,mySensor.readFloatHumidity());
        lpp.addBarometricPressure(3,mySensor.readFloatPressure()/100); // hPa
        LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
#else
        uint16_t Temp=mySensor.readTempC()*100+30000;
        //uint16_t Temp=-12.34*100+30000;
        Sprint(F("T: "));
        Sprintln(Temp);
        mydata[0]=(Temp >> 8) & 0xFF;
        mydata[1]=Temp & 0xFF;

        Temp=mySensor.readFloatHumidity()*100;
        Sprint(F("T: "));
        Sprintln(Temp);
        mydata[2]=(Temp >> 8) & 0xFF;
        mydata[3]=Temp & 0xFF;

        Temp=mySensor.readFloatPressure()/10;
        Sprint(F("T: "));
        Sprintln(Temp);
        mydata[4]=(Temp >> 8) & 0xFF;
        mydata[5]=Temp & 0xFF;
        
        LMIC_setTxData2(1, mydata, 6, 0);
#endif // CAYENNE
        
        //LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Sprintln(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
//    pinMode(13, OUTPUT); 
    while (!Serial); // wait for Serial to be initialized
    Serial.begin(115200);
    delay(100);     // per sample code on RF_95 test

    Wire.begin();

    //***Driver settings********************************//
    //commInterface can be I2C_MODE or SPI_MODE
    //specify chipSelectPin using arduino pin names
    //specify I2C address.  Can be 0x77(default) or 0x76

    //For I2C, enable the following and disable the SPI section
    mySensor.settings.commInterface = I2C_MODE;
    //mySensor.settings.I2CAddress = 0x77;
    mySensor.settings.I2CAddress = 0x76;

    //For SPI enable the following and dissable the I2C section
    //mySensor.settings.commInterface = SPI_MODE;
    //mySensor.settings.chipSelectPin = 10;


    //***Operation settings*****************************//

    //runMode can be:
    //  0, Sleep mode
    //  1 or 2, Forced mode
    //  3, Normal mode
    mySensor.settings.runMode = 1; //Forced mode

    //tStandby can be:
    //  0, 0.5ms
    //  1, 62.5ms
    //  2, 125ms
    //  3, 250ms
    //  4, 500ms
    //  5, 1000ms
    //  6, 10ms
    //  7, 20ms
    mySensor.settings.tStandby = 0;

    //filter can be off or number of FIR coefficients to use:
    //  0, filter off
    //  1, coefficients = 2
    //  2, coefficients = 4
    //  3, coefficients = 8
    //  4, coefficients = 16
    mySensor.settings.filter = 0;

    //tempOverSample can be:
    //  0, skipped
    //  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
    mySensor.settings.tempOverSample = 1;

    //pressOverSample can be:
    //  0, skipped
    //  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
    mySensor.settings.pressOverSample = 1;

    //humidOverSample can be:
    //  0, skipped
    //  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
    mySensor.settings.humidOverSample = 1;
    delay(10);  //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.
    
    auto beginval=mySensor.begin();
    Sprint(F("Starting BME280... result of .begin(): 0x"));
    Sprintln(beginval);
    if (beginval == false) //Begin communication over I2C
       {
        Sprintln(F("The sensor did not respond. Please check wiring."));
        while(1); //Freeze
       }
    
    Sprintln(F("Starting"));

    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x13, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x13, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
    
}
