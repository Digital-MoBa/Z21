/*
  z21.h - library for Z21 mobile protocoll
  Copyright (c) 2013-2022 Philipp Gahtow  All right reserved.

  ROCO Z21 LAN Protocol for Arduino.

  Notice:
    - analyse the data and give back the content and a answer

  Grundlage: Z21 LAN Protokoll Spezifikation V1.12

  Änderungen:
    - 23.09.15 Anpassung LAN_LOCONET_DETECTOR
               Fehlerbeseitigung bei der LAN Prüfsumme
               Anpassung LAN_LOCONET_DISPATCH
    - 14.07.16 add S88 Gruppenindex for request
    - 22.08.16 add POM read notify
    - 19.12.16 add CV return value for Service Mode
    - 27.12.16 add CV no ACK and CV Short Circuit
    - 15.03.17 add System Information
    - 03.04.17 fix LAN_X_SET_LOCO_FUNCTION in DB3 type and index
    - 14.04.17 add EEPROM and store Z21 configuration (EEPROM: 50-75)
    - 19.06.17 add FW Version 1.28, 1.29 and 1.30
    - 06.08.17 add support for Arduino DUE
    - 27.08.17 fix speed step setting
    - 09.01.18 adjust POM write Byte and Bit
    - 21.01.18 adjust notifyz21LNdispatch to long Adr
    - 04.02.18 fix LocoNet dispatch
    - 22.10.18 add loco busy return
    - 02.11.18 adjust returnLocoStateFull with addition non broadcast when requestion only
  information
    - 04.11.18 fix EthSend handel of message with client == 0 and Broadcast with client and without
    - 10.05.20 add message LAN_GET_LOCOMODE and LAN_GET_TURNOUTMODE
    - 04.08.20 fix POM set CV result
    - 17.12.20 add support for ESP32 and adjust ESP8266
    - 01.03.21 set default EEPROM MainV and ProgV to 20V
    - 03.03.21 add new Z21 spezifications v1.10
    - 21.03.21 add request for client identification (ip-hash) and store this plus BC-Flag in EEPROM
    - 30.03.21 fix connecting problem WDP with reporting railpower when request status
    - 11.06.21 add NVS on ESP32 to store EEPROM data
    - 30.09.21 fix storage data in Ethsend with correct datalength; fix problem with
  LAN_X_GET_TURNOUT_INFO to return feedbacks also when LAN_X_SET_TURNOUT is called!
    - 06.11.21 fix EEPROM store BCFlags with IP Hash value. Use EEPROM value from 512 up to 736.
    - 16.11.21 add sending data to client only with "Z21bcNone", if client set and BC-Flag set (not
  "Z21bcNone") then don't inform the client!
    - 14.12.21 limit max packet size for Z21 LocoNet tunnel data to 20 bytes!
    - 03.02.22 fix setCANDetector() data values with 16bit
    - 10.02.22 add LAN_X_CV_POM_ACCESSORY statements
    - 24.04.22 add SystemState.Capabilities for feature report to clients (FW Version 1.42)
               modify LAN_X_LOCO_INFO to FW Version 1.42
    - 25.04.22 add LAN_X_SET_LOCO_FUNCTION_GROUP and LAN_X_SET_LOCO_BINARY_STATE
               fix SET_EXT_ACCESSORY and EXT_ACCESSORY_INFO
    - 29.04.22 add WLANMaus CV Read and write special functions
*/

// include types & constants of Wiring core API
#if defined(WIRING)
#  include <Wiring.h>
#elif ARDUINO >= 100
#  include <Arduino.h>
#else
#  include <WProgram.h>
#endif

//--------------------------------------------------------------
#define z21Port 21105 // local port to listen on

//**************************************************************
// #define SERIALDEBUG //Serial Debug

#if defined(SERIALDEBUG)
#  define ZDebug Serial // Port for the Debugging
#endif

//**************************************************************
// Firmware-Version der Z21:
#define z21FWVersionMSB 0x01
#define z21FWVersionLSB 0x42
/*
HwType:
#define D_HWT_Z21_OLD 0x00000200 // „schwarze Z21” (Hardware-Variante ab 2012)
#define D_HWT_Z21_NEW 0x00000201 // „schwarze Z21”(Hardware-Variante ab 2013)
#define D_HWT_SMARTRAIL 0x00000202 // SmartRail (ab 2012)
#define D_HWT_z21_SMALL 0x00000203 // „weiße z21” Starterset-Variante (ab 2013)
#define D_HWT_z21_START 0x00000204 // „z21 start” Starterset-Variante (ab 2016)
#define D_HWT_Z21_XL 0x00000211 // 10870 „Z21 XL Series” (ab 2020)
#define D_HWT_SINGLE_BOOSTER 0x00000205 // 10806 „Z21 Single Booster” (zLink)
#define D_HWT_DUAL_BOOSTER 0x00000206 // 10807 „Z21 Dual Booster” (zLink)
#define D_HWT_Z21_SWITCH_DECODER 0x00000301 // 10836 „Z21 SwitchDecoder” (zLink)
#define D_HWT_Z21_SIGNAL_DECODER 0x00000302 // 10836 „Z21 SignalDecoder” (zLink)
*/
// Hardware-Typ: 0x00000211 // 10870 „Z21 XL Series” (ab 2020)
#define z21HWTypeMSB 0x02
#define z21HWTypeLSB 0x11
// Seriennummer inside EEPROM:
#define CONFz21SnMSB 0 // 0x01
#define CONFz21SnLSB 1 // 0xE8
//**************************************************************
// Store Z21 configuration inside EEPROM:
#define CONF1STORE      50    //(10x Byte) - Prog, RailCom, etc.
#define CONF2STORE      60    //(15x Byte) - Voltage: Prog, Rail, etc.
#define CLIENTHASHSTORE 0x200 // 512 Start where Client-Hash is stored

//--------------------------------------------------------------
// certain global XPressnet status indicators:
#define csNormal          0x00 // Normal Operation Resumed ist eingeschaltet
#define csEmergencyStop   0x01 // Der Nothalt ist eingeschaltet
#define csTrackVoltageOff 0x02 // Die Gleisspannung ist abgeschaltet
#define csShortCircuit    0x04 // Kurzschluss
#define csServiceMode     0x08 // Der Programmiermodus ist aktiv - Service Mode
// Bitmask CentralStateEx:
#define cseHighTemperature      0x01 // zu hohe Temperatur
#define csePowerLost            0x02 // zu geringe Eingangsspannung
#define cseShortCircuitExternal 0x04 // am externen Booster-Ausgang
#define cseShortCircuitInternal 0x08 // am Hauptgleis oder Programmiergleis

//--------------------------------------------------------------
#define z21clientMAX  30   // Speichergröße für IP-Adressen
#define z21ActTimeIP  20   // Aktivhaltung einer IP für (sec./2)
#define z21IPinterval 2000 // interval at milliseconds

// DCC Speed Steps
#define DCCSTEP14  0x01
#define DCCSTEP28  0x02
#define DCCSTEP128 0x03

struct TypeActIP
{
  byte client;  // Byte client
  byte BCFlag;  // BoadCastFlag - see Z21type.h
  byte time;    // Zeit
  uint16_t adr; // Loco control Adr
};

// library interface description
class z21Class
{
  // user-accessible "public" interface
public:
  z21Class(void); // Constuctor

  void receive(uint8_t client, uint8_t *packet); // Prüfe auf neue Ethernet Daten

  void setPower(byte state); // Zustand Gleisspannung Melden
  byte getPower();           // Zusand Gleisspannung ausgeben

  void setCVPOMBYTE(uint16_t CVAdr, uint8_t value); // POM write byte return

  void setLocoStateExt(int Adr);         // send Loco state to BC
  unsigned long getz21BcFlag(byte flag); // Convert local stored flag back into a Z21 Flag

  void setS88Data(byte *data); // return state of S88 sensors

  void setLNDetector(uint8_t client, byte *data, byte DataLen); // return state from LN detector
  bool setLNMessage(byte *data, byte DataLen, byte bcType, bool TX); // return LN Message

  void setCANDetector(uint16_t NID, uint16_t Adr, uint8_t port, uint8_t typ, uint16_t v1,
                      uint16_t v2); // state from CAN detector

  void setTrntInfo(uint16_t Adr, bool State); // Return the state of accessory

  void setExtACCInfo(uint16_t Adr, byte State, bool Status = 0x00); // Return EXT Accessory INFO

  void setCVReturn(uint16_t CV, uint8_t value); // Return CV Value for Programming
  void setCVNack();                             // Return no ACK from Decoder
  void setCVNackSC();                           // Return Short while Programming

  void sendSystemInfo(
    byte client, uint16_t maincurrent, uint16_t mainvoltage,
    uint16_t temp); // Send to all clients that request via BC the System Information

  // library-accessible "private" interface
private:
  // Variables:
  byte Railpower;                // state of the railpower
  long z21IPpreviousMillis;      // will store last time of IP decount updated
  TypeActIP ActIP[z21clientMAX]; // Speicherarray für IPs

  // Functions:
  void returnLocoStateFull(byte client, uint16_t Adr, bool bc); // Antwort auf Statusabfrage
  void EthSend(byte client, unsigned int DataLen, unsigned int Header, byte *dataString,
               boolean withXOR, byte BC);
  byte getLocalBcFlag(unsigned long flag); // Convert Z21 LAN BC flag to local stored flag
  void clearIP(byte pos);                  // delete the stored client
  void clearIPSlots();                     // delete all stored clients
  void clearIPSlot(byte client);           // delete a client
  byte addIPToSlot(byte client, byte BCFlag);

  void setOtherSlotBusy(byte slot);
  void addBusySlot(byte client, uint16_t adr);
  void reqLocoBusy(uint16_t adr);

  byte getEEPROMBCFlagIndex();                    // return the length of BC-Flag store
  void setEEPROMBCFlag(byte IPHash, byte BCFlag); // add BC-Flag to store
  byte findEEPROMBCFlag(byte IPHash);             // read the BC-Flag for this client

  uint8_t LAST_EXTACC_msg = 0x00;    // for LAN_X_GET_EXT_ACCESSORY_INFO
  bool LAST_EXTACC_received = false; // already had any EXTACC Message?
};

#if defined(__cplusplus)
extern "C"
{
#endif

  extern void notifyz21getSystemInfo(uint8_t client) __attribute__((weak));

  extern void notifyz21EthSend(uint8_t client, uint8_t *data) __attribute__((weak));

  extern void notifyz21LNdetector(uint8_t client, uint8_t typ, uint16_t Adr) __attribute__((weak));
  extern uint8_t notifyz21LNdispatch(uint16_t Adr) __attribute__((weak));
  extern void notifyz21LNSendPacket(uint8_t *data, uint8_t length) __attribute__((weak));

  extern void notifyz21CANdetector(uint8_t client, uint8_t typ, uint16_t ID) __attribute__((weak));

  extern void notifyz21RailPower(uint8_t State) __attribute__((weak));

  extern void notifyz21CVREAD(uint8_t cvAdrMSB, uint8_t cvAdrLSB) __attribute__((weak));
  extern void notifyz21CVWRITE(uint8_t cvAdrMSB, uint8_t cvAdrLSB, uint8_t value)
    __attribute__((weak));
  extern void notifyz21CVPOMWRITEBYTE(uint16_t Adr, uint16_t cvAdr, uint8_t value)
    __attribute__((weak));
  extern void notifyz21CVPOMWRITEBIT(uint16_t Adr, uint16_t cvAdr, uint8_t value)
    __attribute__((weak));
  extern void notifyz21CVPOMREADBYTE(uint16_t Adr, uint16_t cvAdr) __attribute__((weak));
  extern void notifyz21CVPOMACCWRITEBYTE(uint16_t Adr, uint16_t cvAdr, uint8_t value)
    __attribute__((weak));
  extern void notifyz21CVPOMACCWRITEBIT(uint16_t Adr, uint16_t cvAdr, uint8_t value)
    __attribute__((weak));
  extern void notifyz21CVPOMACCREADBYTE(uint16_t Adr, uint16_t cvAdr) __attribute__((weak));

  extern uint8_t notifyz21AccessoryInfo(uint16_t Adr) __attribute__((weak));
  extern void notifyz21Accessory(uint16_t Adr, bool state, bool active) __attribute__((weak));
  extern void notifyz21ExtAccessory(uint16_t Adr, byte state) __attribute__((weak));

  extern void notifyz21LocoState(uint16_t Adr, uint8_t data[]) __attribute__((weak));
  extern void notifyz21LocoFkt(uint16_t Adr, uint8_t type, uint8_t fkt) __attribute__((weak));
  extern void notifyz21LocoFkt0to4(uint16_t Adr, uint8_t fkt) __attribute__((weak));
  extern void notifyz21LocoFkt5to8(uint16_t Adr, uint8_t fkt) __attribute__((weak));
  extern void notifyz21LocoFkt9to12(uint16_t Adr, uint8_t fkt) __attribute__((weak));
  extern void notifyz21LocoFkt13to20(uint16_t Adr, uint8_t fkt) __attribute__((weak));
  extern void notifyz21LocoFkt21to28(uint16_t Adr, uint8_t fkt) __attribute__((weak));
  extern void notifyz21LocoFkt29to36(uint16_t Adr, uint8_t fkt) __attribute__((weak));
  extern void notifyz21LocoFkt37to44(uint16_t Adr, uint8_t fkt) __attribute__((weak));
  extern void notifyz21LocoFkt45to52(uint16_t Adr, uint8_t fkt) __attribute__((weak));
  extern void notifyz21LocoFkt53to60(uint16_t Adr, uint8_t fkt) __attribute__((weak));
  extern void notifyz21LocoFkt61to68(uint16_t Adr, uint8_t fkt) __attribute__((weak));
  extern void notifyz21LocoFktExt(uint16_t Adr, uint8_t low, uint8_t high) __attribute__((weak));
  extern void notifyz21LocoSpeed(uint16_t Adr, uint8_t speed, uint8_t steps) __attribute__((weak));

  extern void notifyz21S88Data(uint8_t gIndex)
    __attribute__((weak)); // return last state S88 Data for the Client!

  extern uint16_t notifyz21Railcom() __attribute__((weak)); // return global Railcom Adr

  extern void notifyz21UpdateConf()
    __attribute__((weak)); // information for DCC via EEPROM (RailCom, ProgMode,...)

  extern uint8_t notifyz21ClientHash(uint8_t client) __attribute__((weak));

#if defined(__cplusplus)
}
#endif
