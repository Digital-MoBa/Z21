/*
  z21nvs.h - library for emulate EEPROM on Non-Volatile Storage
  Copyright (c) 2021 Philipp Gahtow  All right reserved.
*/

// include types & constants of Wiring core API
#if defined(WIRING)
 #include <Wiring.h>
#elif ARDUINO >= 100
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Preferences.h>

#define EESize 512    //Größe des EEPROM

// library interface description
class z21nvsClass
{
  // user-accessible "public" interface
  public:
	z21nvsClass(void);	//Constuctor
	
	bool begin(uint16_t size);
	
	uint8_t read(uint16_t adr);
	bool write(uint16_t adr, uint8_t value);
	
	bool commit(void);
	
/*	
  // library-accessible "private" interface
  private:
  
	//uint8_t storage[EESize];
*/
};