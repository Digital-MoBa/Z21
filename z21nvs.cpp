/*
*****************************************************************************
*		z21nvs.cpp - library for emulate EEPROM on Non-Volatile Storage
*		Copyright (c) 2021 Philipp Gahtow  All right reserved.
*
*
*****************************************************************************
* IMPORTANT:
*
* 	Please contact ROCO Inc. for more details.
*****************************************************************************
*/

// include this library's description file
#include <z21nvs.h>

Preferences store;

// Constructor /////////////////////////////////////////////////////////////////
// Function that handles the creation and setup of instances

z21nvsClass::z21nvsClass()
{
  // Open Preferences with my-app namespace. Each application module, library, etc
  // has to use a namespace name to prevent key name collisions. We will open storage in
  // RW-mode (second parameter has to be false).
  // Note: Namespace name is limited to 15 chars.
  // store.begin("Z21-app", false);
}

// Public Methods //////////////////////////////////////////////////////////////
// Functions available in Wiring sketches, this library, and other libraries

//*********************************************************************************************
// Daten ermitteln
uint8_t z21nvsClass::read(uint16_t adr)
{
  store.begin("Z21-app", true);

  // size_t schLen = store.getBytes("Z21EEPROM", NULL, NULL);
  char buffer[EESize];
  store.getBytes("Z21EEPROM", buffer, EESize);

  // Close the Preferences
  store.end();

  if (adr < EESize)
    return buffer[adr];
  return 0xFF;
}

bool z21nvsClass::write(uint16_t adr, uint8_t value)
{
  store.begin("Z21-app", false);

  // size_t schLen = store.getBytes("Z21EEPROM", NULL, NULL);
  char buffer[EESize];
  store.getBytes("Z21EEPROM", buffer, EESize);

  if (adr < EESize)
  {
    buffer[adr] = value;
    store.putBytes("Z21EEPROM", buffer, EESize);
    // Close the Preferences
    store.end();

    return true;
  }
  else
  {
    // Close the Preferences
    store.end();
    return false;
  }
}

bool z21nvsClass::begin(uint16_t size)
{
  return true;
}

bool z21nvsClass::commit(void)
{
  return true;
}
