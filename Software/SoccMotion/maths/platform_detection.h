#elif TEENSYDUINO
    #ifdef ARDUINO_TEENSY40
        #define TEENSY_4_0
        #define EEPROM_MAX_SIZE     1080
    #elif ARDUINO_TEENSY41
        #define TEENSY_4_1
        #define EEPROM_MAX_SIZE     4096  // Teensy 4.1 EEPROM emulation size
    #endif