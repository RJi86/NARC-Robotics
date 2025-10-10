#pragma once

#ifdef ARDUINO
	#ifdef ARDUINO_ARCH_AVR
		#ifdef ARDUINO_AVR_MEGA2560
			#define ARDUINO_MEGA2560
			#define EEPROM_MAX_SIZE		4096
		#elif ARDUINO_AVR_ATmega328
			#define ARDUINO_ATMEGA328
			#define EEPROM_MAX_SIZE		1024
		#endif
	#elif TEENSYDUINO
		#ifdef ARDUINO_TEENSY40
			#define TEENSY_4_0
			#define EEPROM_MAX_SIZE		1080
		#elif ARDUINO_TEENSY41
			#define TEENSY_4_1
			#define EEPROM_MAX_SIZE		4096  // Teensy 4.1 EEPROM emulation size
		#endif
	#endif
#endif

#ifdef ARDUINO_ATMEGA328
typedef uint32_t ssize_t;
#endif