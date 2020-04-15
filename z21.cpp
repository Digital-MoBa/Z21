/*
*****************************************************************************
  *		z21.cpp - library for Roco Z21 LAN protocoll
  *		Copyright (c) 2013-2019 Philipp Gahtow  All right reserved.
  *
  *
*****************************************************************************
  * IMPORTANT:
  * 
  * 	Please contact ROCO Inc. for more details.
*****************************************************************************
*/

// include this library's description file
#include <z21.h>
#include <z21header.h>

#if defined(__arm__)
#include <DueFlashStorage.h>
DueFlashStorage FlashStore;
#define FSTORAGE 	FlashStore
#define FSTORAGEMODE write
#else
// AVR based Boards follows
#include <EEPROM.h>
#define FSTORAGE 	EEPROM
	#if defined(ARDUINO_ESP8266_ESP01) || defined(ARDUINO_ESP8266_WEMOS_D1MINI) //ESP8266 or WeMos
		#define FSTORAGEMODE write
	#else
		#define FSTORAGEMODE update
	#endif
#endif

// Constructor /////////////////////////////////////////////////////////////////
// Function that handles the creation and setup of instances

z21Class::z21Class()
{
	// initialize this instance's variables 
    z21IPpreviousMillis = 0;
    Railpower = csTrackVoltageOff;
	clearIPSlots();
}

// Public Methods //////////////////////////////////////////////////////////////
// Functions available in Wiring sketches, this library, and other libraries

//*********************************************************************************************
//Daten ermitteln und Auswerten
void z21Class::receive(uint8_t client, uint8_t *packet) 
{
	addIPToSlot(client, 0);
	// send a reply, to the IP address and port that sent us the packet we received
	int header = (packet[3]<<8) + packet[2];
	byte data[16]; 			//z21 send storage
		
	switch (header) {
		case LAN_GET_SERIAL_NUMBER:
		  #if defined(SERIALDEBUG)
		  ZDebug.println("GET_SERIAL_NUMBER");  
		  #endif
		  data[0] = z21SnLSB;
		  data[1] = z21SnMSB;
		  data[2] = 0x00; 
		  data[3] = 0x00;
		  EthSend(client, 0x08, LAN_GET_SERIAL_NUMBER, data, false, Z21bcNone); //Seriennummer 32 Bit (little endian)
		  break; 
		case LAN_GET_HWINFO:
		  #if defined(SERIALDEBUG)
		  ZDebug.println("GET_HWINFO"); 
		  #endif
		  data[0] = z21HWTypeLSB;  //HwType 32 Bit
		  data[1] = z21HWTypeMSB;
		  data[2] = 0x00; 
		  data[3] = 0x00;
		  data[4] = z21FWVersionLSB;  //FW Version 32 Bit
		  data[5] = z21FWVersionMSB;
		  data[6] = 0x00; 
		  data[7] = 0x00;
		  EthSend (client, 0x0C, LAN_GET_HWINFO, data, false, Z21bcNone);
		  break;  
		case LAN_LOGOFF:
		  #if defined(SERIALDEBUG)
		  ZDebug.println("LOGOFF");
		  #endif
		  clearIPSlot(client);
		  //Antwort von Z21: keine
		  break; 
		case LAN_GET_CODE:	//SW Feature-Umfang der Z21   
		  /*#define Z21_NO_LOCK        0x00  // keine Features gesperrt 
			#define z21_START_LOCKED   0x01  // �z21 start�: Fahren und Schalten per LAN gesperrt 
			#define z21_START_UNLOCKED 0x02  // �z21 start�: alle Feature-Sperren aufgehoben */
		  data[0] = 0x00; //keine Features gesperrt
		  EthSend (client, 0x05, LAN_GET_CODE, data, false, Z21bcNone);	
		  break;
		case (LAN_X_Header):
		  //---------------------- LAN X-Header BEGIN ---------------------------	
		  switch (packet[4]) { //X-Header
		  case LAN_X_GET_SETTING: 
			//---------------------- Switch BD0 BEGIN ---------------------------	
			switch (packet[5]) {  //DB0
			case 0x21:
			  #if defined(SERIALDEBUG)
			  ZDebug.println("X_GET_VERSION"); 
			  #endif
			  data[0] = LAN_X_GET_VERSION;	//X-Header: 0x63
			  data[1] = 0x21;	//DB0
			  data[2] = 0x30;   //X-Bus Version
			  data[3] = 0x12;  //ID der Zentrale
			  EthSend (client, 0x09, LAN_X_Header, data, true, Z21bcNone);
			  break;
			case 0x24:
			  data[0] = LAN_X_STATUS_CHANGED;	//X-Header: 0x62
			  data[1] = 0x22;			//DB0
			  data[2] = Railpower;		//DB1: Status
			  //ZDebug.print("X_GET_STATUS "); 
				  //csEmergencyStop  0x01 // Der Nothalt ist eingeschaltet 
				  //csTrackVoltageOff  0x02 // Die Gleisspannung ist abgeschaltet 
				  //csShortCircuit  0x04 // Kurzschluss 
				  //csProgrammingModeActive 0x20 // Der Programmiermodus ist aktiv 
			  EthSend (client, 0x08, LAN_X_Header, data, true, Z21bcNone);
			  break;
			case 0x80:
			  #if defined(SERIALDEBUG)
			  ZDebug.println("X_SET_TRACK_POWER_OFF");
			  #endif
			  if (notifyz21RailPower)
				notifyz21RailPower(csTrackVoltageOff);
			  break;
			case 0x81:
			  #if defined(SERIALDEBUG)
			  ZDebug.println("X_SET_TRACK_POWER_ON");
			  #endif
			  if (notifyz21RailPower)
				notifyz21RailPower(csNormal);
			  break;  
			}
			//---------------------- Switch DB0 ENDE ---------------------------	
			break;  //ENDE DB0
		  case LAN_X_CV_READ:
			if (packet[5] == 0x11) {  //DB0
			  #if defined(SERIALDEBUG)
			  ZDebug.println("X_CV_READ"); 
			  #endif
			  if (notifyz21CVREAD)
				notifyz21CVREAD(packet[6], packet[7]); //CV_MSB, CV_LSB
			}
			break;             
		  case LAN_X_CV_WRITE: 
			if (packet[5] == 0x12) {  //DB0
			  #if defined(SERIALDEBUG)
			  ZDebug.println("X_CV_WRITE"); 
			  #endif
			  if (notifyz21CVWRITE)
				notifyz21CVWRITE(packet[6], packet[7], packet[8]); //CV_MSB, CV_LSB, value
			}
			break;
		  case LAN_X_CV_POM: 
			if (packet[5] == 0x30) {  //DB0
			  uint16_t Adr = ((packet[6] & 0x3F) << 8) + packet[7];
			  uint16_t CVAdr = ((packet[8] & B11) << 8) + packet[9]; 
			  byte value = packet[10];
			  if ((packet[8] & 0xFC) == 0xEC) {
				#if defined(SERIALDEBUG)
				ZDebug.println("LAN_X_CV_POM_WRITE_BYTE"); 
				#endif
				if (notifyz21CVPOMWRITEBYTE)
					notifyz21CVPOMWRITEBYTE (Adr, CVAdr, value);  //set Byte
			  }
			  else if ((packet[8] & 0xFC) == 0xE8) {
				#if defined(SERIALDEBUG)
				ZDebug.println("LAN_X_CV_POM_WRITE_BIT"); 
				#endif
				if (notifyz21CVPOMWRITEBIT)
					notifyz21CVPOMWRITEBIT (Adr, CVAdr, value);  //set Bit
			  }
			  else {
				  #if defined(SERIALDEBUG)
				  ZDebug.println("LAN_X_CV_POM_READ_BYTE"); 
				  #endif
				  if (notifyz21CVPOMREADBYTE)
					notifyz21CVPOMREADBYTE (Adr, CVAdr);  //read byte
			  }
			}
			else if (packet[5] == 0x31) {  //DB0
			  #if defined(SERIALDEBUG)
			  ZDebug.println("LAN_X_CV_POM_ACCESSORY"); 
			  #endif
			}
			break;      
		  case LAN_X_GET_TURNOUT_INFO: {
			#if defined(SERIALDEBUG)
			  ZDebug.print("X_GET_TURNOUT_INFO ");
			#endif
			  if (notifyz21AccessoryInfo) {
				  data[0] = 0x43;  //X-HEADER
				  data[1] = packet[5]; //High
				  data[2] = packet[6]; //Low
				  if (notifyz21AccessoryInfo((packet[5] << 8) + packet[6]) == true)
					  data[3] = 0x02;  //active
				  else data[3] = 0x01;  //inactive
			      EthSend (client, 0x09, LAN_X_Header, data, true, Z21bcNone);    //BC new 23.04. !!!(old = 0)
			  }
			  break;
		  }
		  case LAN_X_SET_TURNOUT: {
			#if defined(SERIALDEBUG)
			ZDebug.print("X_SET_TURNOUT Adr.:");
			ZDebug.print((packet[5] << 8) + packet[6]);
			ZDebug.print(":");
			ZDebug.print(bitRead(packet[7], 0));
			ZDebug.print("-");
			ZDebug.println(bitRead(packet[7], 3));
			#endif
			//bool TurnOnOff = bitRead(packet[7],3);  //Spule EIN/AUS
			if (notifyz21Accessory) {
				notifyz21Accessory((packet[5] << 8) + packet[6], bitRead(packet[7], 0), bitRead(packet[7], 3));
			}						//	Addresse					Links/Rechts			Spule EIN/AUS
			break;  
		  }
		  case LAN_X_SET_STOP:
			#if defined(SERIALDEBUG)
			ZDebug.println("X_SET_STOP");
			#endif
			if (notifyz21RailPower)
				notifyz21RailPower(csEmergencyStop);
			break;  
		  case LAN_X_GET_LOCO_INFO:
			if (packet[5] == 0xF0) {  //DB0
			  //ZDebug.print("X_GET_LOCO_INFO: ");
			  //Antwort: LAN_X_LOCO_INFO  Adr_MSB - Adr_LSB
			  returnLocoStateFull(client, word(packet[6] & 0x3F, packet[7]), false);	
			}
			break;  
		  case LAN_X_SET_LOCO:
			//setLocoBusy:
			addBusySlot(client,word(packet[6] & 0x3F, packet[7]));
			
			if (packet[5] == LAN_X_SET_LOCO_FUNCTION) {  //DB0
			  //LAN_X_SET_LOCO_FUNCTION  Adr_MSB        Adr_LSB            Type (00=AUS/01=EIN/10=UM)      Funktion
			  if (notifyz21LocoFkt)
				notifyz21LocoFkt(word(packet[6] & 0x3F, packet[7]), packet[8] >> 6, packet[8] & B00111111); 
			  //uint16_t Adr, uint8_t type, uint8_t fkt
			}
			else {  //DB0
				  //ZDebug.print("X_SET_LOCO_DRIVE ");
				  byte steps = 128;	//default value S=3; DCC 128 Fahrstufen
				  if (packet[5] == 0x12)	//S=2; DCC 28 Fahrstufen
					steps = 28;
				  else if (packet[5] == 0x10)	//S=0; DCC 14 Fahrstufen
					steps = 14;
				if (notifyz21LocoSpeed)
					notifyz21LocoSpeed(word(packet[6] & 0x3F, packet[7]), packet[8],steps);
			}
			returnLocoStateFull(client, word(packet[6] & 0x3F, packet[7]), true);	
			break;  
		  case LAN_X_GET_FIRMWARE_VERSION:
			#if defined(SERIALDEBUG)
			ZDebug.println("X_GET_FIRMWARE_VERSION"); 
			#endif
			data[0] = 0xF3;		//identify Firmware (not change)
			data[1] = 0x0A;		//identify Firmware (not change)
			data[2] = z21FWVersionMSB;   //V_MSB
			data[3] = z21FWVersionLSB;  //V_LSB
			EthSend (client, 0x09, LAN_X_Header, data, true, Z21bcNone);
			break;     
		  case 0x73:
			//LAN_X_??? WLANmaus periodische Abfrage: 
			//0x09 0x00 0x40 0x00 0x73 0x00 0xFF 0xFF 0x00
			//length X-Header	XNet-Msg			  speed?
			#if defined(SERIALDEBUG)
			ZDebug.println("LAN-X_WLANmaus"); 
			#endif
			//set Broadcastflags for WLANmaus:
			if (addIPToSlot(client, 0x00) == 0)
				addIPToSlot(client, Z21bcAll);
			break;
		  default:
			#if defined(SERIALDEBUG)
			ZDebug.print("UNKNOWN_LAN-X_COMMAND"); 
			//for (byte i = 0; i < packet[0]; i++) {
			//	ZDebug.print(" 0x");
			//	ZDebug.print(packet[i], HEX);
			//}
			ZDebug.println();
			#endif
			data[0] = 0x61;
			data[1] = 0x82;
			EthSend (client, 0x07, LAN_X_Header, data, true, Z21bcNone);
		  }
		  //---------------------- LAN X-Header ENDE ---------------------------	
		  break; 
		case (LAN_SET_BROADCASTFLAGS): {
			unsigned long bcflag = packet[7];
			bcflag = packet[6] | (bcflag << 8);
			bcflag = packet[5] | (bcflag << 8);
			bcflag = packet[4] | (bcflag << 8);
			addIPToSlot(client, getLocalBcFlag(bcflag));
			//no inside of the protokoll, but good to have:
			if (notifyz21RailPower)
				notifyz21RailPower(Railpower); //Zustand Gleisspannung Antworten
			#if defined(SERIALDEBUG)
			  ZDebug.print("SET_BROADCASTFLAGS: "); 
			  ZDebug.println(addIPToSlot(client, 0x00), BIN);
			  // 1=BC Power, Loco INFO, Trnt INFO; 2=BC �nderungen der R�ckmelder am R-Bus
			#endif
			break;
		  }
		case (LAN_GET_BROADCASTFLAGS): {
			unsigned long flag = getz21BcFlag(addIPToSlot(client, 0x00));  
			data[0] = flag;
			data[1] = flag >> 8;
			data[2] = flag >> 16;
			data[3] = flag >> 24;
			EthSend (client, 0x08, LAN_GET_BROADCASTFLAGS, data, false, Z21bcNone); 
			#if defined(SERIALDEBUG)
			  ZDebug.print("GET_BROADCASTFLAGS: ");
			  ZDebug.println(flag, BIN);
			#endif
			break;
		  }
		case (LAN_GET_LOCOMODE):
		break;
		case (LAN_SET_LOCOMODE):
		break;
		case (LAN_GET_TURNOUTMODE):
		break;
		case (LAN_SET_TURNOUTMODE):
		break;
		case (LAN_RMBUS_GETDATA):
			  if (notifyz21S88Data) {
				#if defined(SERIALDEBUG)
				  ZDebug.println("RMBUS_GETDATA");
				#endif
				//ask for group state 'Gruppenindex'
				notifyz21S88Data(packet[4]);	//normal Antwort hier nur an den anfragenden Client! (Antwort geht hier an alle!)
			  }
			  break;
		case (LAN_RMBUS_PROGRAMMODULE):
		break;
		case (LAN_SYSTEMSTATE_GETDATA): {	//System state
			  #if defined(SERIALDEBUG)
			  ZDebug.println("LAN_SYS-State");
			  #endif
			  if (notifyz21getSystemInfo) 
				  notifyz21getSystemInfo(client);
			break;
		}
		case (LAN_RAILCOM_GETDATA): {
			  uint16_t Adr = 0;
			  if (packet[4] == 0x01) {	//RailCom-Daten f�r die gegebene Lokadresse anfordern
				Adr = word(packet[6],packet[5]);
			  }
			  if (notifyz21Railcom)
				  Adr = notifyz21Railcom();	//return global Railcom Adr
			  data[0] = Adr >> 8;	//LocoAddress
			  data[1] = Adr & 0xFF;	//LocoAddress
			  data[2] = 0x00;	//UINT32 ReceiveCounter Empfangsz�hler in Z21 
			  data[3] = 0x00;
			  data[4] = 0x00;
			  data[5] = 0x00;
			  data[6] = 0x00;	//UINT32 ErrorCounter Empfangsfehlerz�hler in Z21
			  data[7] = 0x00;
			  data[8] = 0x00;
			  data[9] = 0x00;
			  /*
			  data[10] = 0x00;	//UINT8 Reserved1 experimentell, siehe Anmerkung 
			  data[11] = 0x00;	//UINT8 Reserved2 experimentell, siehe Anmerkung 
			  data[12] = 0x00;	//UINT8 Reserved3 experimentell, siehe Anmerkung 
			  */
			  EthSend (client, 0x0E, LAN_RAILCOM_DATACHANGED, data, false, Z21bcNone);
			break;  
		}
		case (LAN_LOCONET_FROM_LAN): {
			#if defined(SERIALDEBUG)
			  ZDebug.println("LOCONET_FROM_LAN"); 
			#endif
			if (notifyz21LNSendPacket) {
				byte LNdata[packet[0] - 0x04];  //n Bytes
				for (byte i = 0; i < (packet[0] - 0x04); i++) 
					LNdata[i] = packet[0x04+i];
				notifyz21LNSendPacket(LNdata, packet[0] - 0x04);  
				//Melden an andere LAN-Client das Meldung auf LocoNet-Bus geschrieben wurde
				EthSend(client, packet[0], LAN_LOCONET_FROM_LAN, packet, false, Z21bcLocoNet_s);  //LAN_LOCONET_FROM_LAN
			}
			break;
		}
		case (LAN_LOCONET_DISPATCH_ADDR): {
			if (notifyz21LNdispatch) {
				data[0] = packet[4];
				data[1] = packet[5];
				data[2] = notifyz21LNdispatch(word(packet[5], packet[4]));	//dispatchSlot
				#if defined(SERIALDEBUG)
					ZDebug.print("LOCONET_DISPATCH_ADDR ");
					ZDebug.print(word(packet[5], packet[4]));
					ZDebug.print(",");
					ZDebug.println(data[2]);
				#endif
				EthSend(client, 0x07, LAN_LOCONET_DISPATCH_ADDR, data, false, Z21bcNone);
			}
			break; }
		case (LAN_LOCONET_DETECTOR):
			  if (notifyz21LNdetector) {
				#if defined(SERIALDEBUG)
					ZDebug.println("LOCONET_DETECTOR Abfrage");
				#endif
				notifyz21LNdetector(packet[4], word(packet[6], packet[5]));	//Anforderung Typ & Reportadresse
			  }
			break;
		case (LAN_CAN_DETECTOR):
			if (notifyz21CANdetector) {
				#if defined(SERIALDEBUG)
					ZDebug.println("CAN_DETECTOR Abfrage");
				#endif
				notifyz21CANdetector(packet[4], word(packet[6], packet[5]));	//Anforderung Typ & CAN-ID
			}
			break;
		case (0x12): 	//configuration read
			// <-- 04 00 12 00 	
			// 0e 00 12 00 01 00 01 03 01 00 03 00 00 00
			for (byte i = 0; i < 10; i++) {
				data[i] = FSTORAGE.read(CONF1STORE+i);
			}
			EthSend(client, 0x0e, 0x12, data, false, Z21bcNone);
			#if defined(SERIALDEBUG)
				ZDebug.print("Z21 Eins(read) ");
				ZDebug.print("RailCom: ");
				ZDebug.print(data[0], HEX);
				ZDebug.print(", PWR-Button: ");
				ZDebug.print(data[2], HEX);
				ZDebug.print(", ProgRead: ");
				switch (data[3]) {
					case 0x00: ZDebug.print("nothing"); break;
					case 0x01: ZDebug.print("Bit"); break;
					case 0x02: ZDebug.print("Byte"); break;
					case 0x03: ZDebug.print("both"); break;
				}
				ZDebug.println();
			#endif
			break;
		case (0x13): {	//configuration write
			//<-- 0e 00 13 00 01 00 01 03 01 00 03 00 00 00 
			//0x0e = Length; 0x12 = Header
			/* Daten:
			(0x01) RailCom: 0=aus/off, 1=ein/on
			(0x00)
			(0x01) Power-Button: 0=Gleisspannung aus, 1=Nothalt
			(0x03) Auslese-Modus: 0=Nichts, 1=Bit, 2=Byte, 3=Beides
			*/
			#if defined(SERIALDEBUG)
				ZDebug.print("Z21 Eins(write) ");
				ZDebug.print("RailCom: ");
				ZDebug.print(packet[4], HEX);
				ZDebug.print(", PWR-Button: ");
				ZDebug.print(packet[6], HEX);
				ZDebug.print(", ProgRead: ");
				switch (packet[7]) {
					case 0x00: ZDebug.print("nothing"); break;
					case 0x01: ZDebug.print("Bit"); break;
					case 0x02: ZDebug.print("Byte"); break;
					case 0x03: ZDebug.print("both"); break;
				}
				ZDebug.println();
			#endif
			for (byte i = 0; i < 10; i++) {
				FSTORAGE.FSTORAGEMODE(CONF1STORE+i,packet[4+i]);
			}
			//Request DCC to change
			if (notifyz21UpdateConf)
				notifyz21UpdateConf();
			break;
		}
		case (0x16):  //configuration read
			//<-- 04 00 16 00 
			//14 00 16 00 19 06 07 01 05 14 88 13 10 27 32 00 50 46 20 4e 
			for (byte i = 0; i < 16; i++) {
				data[i] = FSTORAGE.read(CONF2STORE+i);
			}
			EthSend(client, 0x14, 0x16, data, false, Z21bcNone);
			#if defined(SERIALDEBUG)
				ZDebug.print("Z21 Eins(read) ");
				ZDebug.print("RstP(s): ");
				ZDebug.print(data[0]);
				ZDebug.print(", RstP(f): ");
				ZDebug.print(data[1]);
				ZDebug.print(", ProgP: ");
				ZDebug.print(data[2]);
				ZDebug.print(", MainV: ");
				ZDebug.print(word(data[13],data[12]));
				ZDebug.print(", ProgV: ");
				ZDebug.print(word(data[15],data[14]));
				ZDebug.println();
			#endif
			break;
		case (0x17): {	//configuration write
			//<-- 14 00 17 00 19 06 07 01 05 14 88 13 10 27 32 00 50 46 20 4e 
			//0x14 = Length; 0x16 = Header(read), 0x17 = Header(write)
			/* Daten:
			(0x19) Reset Packet (starten) (25-255)
			(0x06) Reset Packet (fortsetzen) (6-64)
			(0x07) Programmier-Packete (7-64)
			(0x01) ?
			(0x05) ?
			(0x14) ?
			(0x88) ?
			(0x13) ?
			(0x10) ?
			(0x27) ?
			(0x32) ?
			(0x00) ?
			(0x50) Hauptgleis (LSB) (12-22V)
			(0x46) Hauptgleis (MSB)
			(0x20) Programmiergleis (LSB) (12-22V): 20V=0x4e20, 21V=0x5208, 22V=0x55F0
			(0x4e) Programmiergleis (MSB)
			*/
			#if defined(SERIALDEBUG)
				ZDebug.print("Z21 Eins(write) ");
				ZDebug.print("RstP(s): ");
				ZDebug.print(packet[4]);
				ZDebug.print(", RstP(f): ");
				ZDebug.print(packet[5]);
				ZDebug.print(", ProgP: ");
				ZDebug.print(packet[6]);
				ZDebug.print(", MainV: ");
				ZDebug.print(word(packet[17],packet[16]));
				ZDebug.print(", ProgV: ");
				ZDebug.print(word(packet[19],packet[18]));
				ZDebug.println();
			#endif
			for (byte i = 0; i < 16; i++) {
				FSTORAGE.FSTORAGEMODE(CONF2STORE+i,packet[4+i]);
			}
			//Request DCC to change
			if (notifyz21UpdateConf)
				notifyz21UpdateConf();
			break;
		}
		default:
		  #if defined(SERIALDEBUG)
			ZDebug.print("UNKNOWN_COMMAND"); 
		//	for (byte i = 0; i < packet[0]; i++) {
		//		ZDebug.print(" 0x");
		//		ZDebug.print(packet[i], HEX);
		//	}
			ZDebug.println();
		  #endif
		  data[0] = 0x61;
		  data[1] = 0x82;
		  EthSend (client, 0x07, LAN_X_Header, data, true, Z21bcNone);
		}
	//---------------------------------------------------------------------------------------
	//check if IP is still used:
	unsigned long currentMillis = millis();
	if ((currentMillis - z21IPpreviousMillis) > z21IPinterval) {
		z21IPpreviousMillis = currentMillis;   
		for (byte i = 0; i < z21clientMAX; i++) {
			if (ActIP[i].time > 0) {
				ActIP[i].time--;    //Zeit herrunterrechnen
			}
			else {
				clearIP(i); 	//clear IP DATA
				//send MESSAGE clear Client
			}
		} 
	}
}

//--------------------------------------------------------------------------------------------
//Zustand der Gleisversorgung setzten
void z21Class::setPower(byte state) 
{
	byte data[] = { LAN_X_BC_TRACK_POWER, 0x00  };
	Railpower = state;
	switch (state) {
		case csNormal: 
				data[1] = 0x01;
				break;
		case csTrackVoltageOff: 
				data[1] = 0x00;
				break;
		case csServiceMode: 
				data[1] = 0x02;
				break;
		case csShortCircuit: 
				data[1] = 0x08;
				break;
		case csEmergencyStop:
				data[0] = 0x81;
				data[1] = 0x00;    
				break;
	}
	EthSend(0, 0x07, LAN_X_Header, data, true, Z21bcAll_s);
	#if defined(SERIALDEBUG)
	ZDebug.print("set_X_BC_TRACK_POWER ");
	ZDebug.println(state, HEX);
	#endif
}
  
//--------------------------------------------------------------------------------------------
//Abfrage letzte Meldung �ber Gleispannungszustand
byte z21Class::getPower() 
{
	return Railpower;
}

//--------------------------------------------------------------------------------------------
//return request for POM read byte
void z21Class::setCVPOMBYTE (uint16_t CVAdr, uint8_t value) {
	byte data[5]; 
				data[0] = 0x64; //X-Header
				data[1] = 0x14; //DB0
				data[2] = (CVAdr >> 8) & 0x3F;  //CV_MSB;
				data[3] = CVAdr & 0xFF; //CV_LSB;
				data[4] = value;
				EthSend (0, 0x0A, LAN_X_Header, data, true, 0x00);
}				


//--------------------------------------------------------------------------------------------
//Zustand R�ckmeldung non - Z21 device - Busy!
void z21Class::setLocoStateExt (int Adr) 
{
	uint8_t ldata[6];
	if (notifyz21LocoState)
		notifyz21LocoState(Adr, ldata); //uint8_t Steps[0], uint8_t Speed[1], uint8_t F0[2], uint8_t F1[3], uint8_t F2[4], uint8_t F3[5]
	
	byte data[9]; 
	data[0] = LAN_X_LOCO_INFO;  //0xEF X-HEADER
	data[1] = (Adr >> 8) & 0x3F;
	data[2] = Adr & 0xFF;
	// Fahrstufeninformation: 0=14, 2=28, 4=128 
	if ((ldata[0] & 0x03) == DCCSTEP14)
		data[3] = 0;	// 14 steps
	if ((ldata[0] & 0x03) == DCCSTEP28)
		data[3] = 2;	// 28 steps
	if ((ldata[0] & 0x03) == DCCSTEP128)		
		data[3] = 4;	// 128 steps
	data[3] = data[3] | 0x08; //BUSY!
		
	data[4] = (char) ldata[1];	//DSSS SSSS
	data[5] = (char) ldata[2];    //F0, F4, F3, F2, F1
	data[6] = (char) ldata[3];    //F5 - F12; Funktion F5 ist bit0 (LSB)
	data[7] = (char) ldata[4];  //F13-F20
	data[8] = (char) ldata[5];  //F21-F28

	reqLocoBusy(Adr);
	EthSend(0, 14, LAN_X_Header, data, true, Z21bcAll_s | Z21bcNetAll_s);  //Send Loco Status und Funktions to all active Apps 
}

//--------------------------------------------------------------------------------------------
//Gibt aktuellen Lokstatus an Anfragenden Zur�ck
void z21Class::returnLocoStateFull (byte client, uint16_t Adr, bool bc) 
//bc = true => to inform also other client over the change.
//bc = false => just ask about the loco state
{
	uint8_t ldata[6];
	if (notifyz21LocoState)
		notifyz21LocoState(Adr, ldata); //uint8_t Steps[0], uint8_t Speed[1], uint8_t F0[2], uint8_t F1[3], uint8_t F2[4], uint8_t F3[5]
	
	byte data[9]; 
	data[0] = LAN_X_LOCO_INFO;  //0xEF X-HEADER
	data[1] = (Adr >> 8) & 0x3F;
	data[2] = Adr & 0xFF;
	// Fahrstufeninformation: 0=14, 2=28, 4=128 
	if ((ldata[0] & 0x03) == DCCSTEP14)
		data[3] = 0;	// 14 steps
	if ((ldata[0] & 0x03) == DCCSTEP28)
		data[3] = 2;	// 28 steps
	if ((ldata[0] & 0x03) == DCCSTEP128)		
		data[3] = 4;	// 128 steps
	data[3] = data[3] | 0x08; //BUSY!
		
	data[4] = (char) ldata[1];	//DSSS SSSS
	data[5] = (char) ldata[2];  //F0, F4, F3, F2, F1
	data[6] = (char) ldata[3];  //F5 - F12; Funktion F5 ist bit0 (LSB)
	data[7] = (char) ldata[4];  //F13-F20
	data[8] = (char) ldata[5];  //F21-F28
	
	//Info to all:
	for (byte i = 0; i < z21clientMAX; i++) {
		if (ActIP[i].client != client) {
			if ((ActIP[i].BCFlag & (Z21bcAll_s | Z21bcNetAll_s)) > 0) {
				if (bc == true)
					EthSend (ActIP[i].client, 14, LAN_X_Header, data, true, Z21bcNone);  //Send Loco status und Funktions to BC Apps
			}
		}
		else { //Info to client that ask:
			if (ActIP[i].adr == Adr) {
				data[3] = data[3] & B111;	//clear busy flag!
			}
			EthSend (client, 14, LAN_X_Header, data, true, Z21bcNone);  //Send Loco status und Funktions to request App
			data[3] = data[3] | 0x08; //BUSY!
		}
	}
	
}



//--------------------------------------------------------------------------------------------
//return state of S88 sensors
void z21Class::setS88Data(byte *data) {	
	EthSend(0, 0x0F, LAN_RMBUS_DATACHANGED, data, false, Z21bcRBus_s); //RMBUS_DATACHANED
}

//--------------------------------------------------------------------------------------------
//return state from LN detector
void z21Class::setLNDetector(byte *data, byte DataLen) {
	EthSend(0, 0x04 + DataLen, LAN_LOCONET_DETECTOR, data, false, Z21bcLocoNetGBM_s);  //LAN_LOCONET_DETECTOR
}

//--------------------------------------------------------------------------------------------
//LN Meldungen weiterleiten
void z21Class::setLNMessage(byte *data, byte DataLen, byte bcType, bool TX) {
	if (TX)   //Send by Z21 or Receive a Packet?
		EthSend(0, 0x04 + DataLen, LAN_LOCONET_Z21_TX, data, false, bcType);  //LAN_LOCONET_Z21_TX
	else EthSend(0, 0x04 + DataLen, LAN_LOCONET_Z21_RX, data, false, bcType);  //LAN_LOCONET_Z21_RX
}

//--------------------------------------------------------------------------------------------
//return state from CAN detector
void z21Class::setCANDetector(uint16_t NID, uint16_t Adr, uint8_t port, uint8_t typ, uint16_t v1, uint16_t v2) {
	byte data[10];
	data[0] = NID & 0x08;
	data[1] = NID >> 8;
	data[2] = Adr & 0x08;
	data[3] = Adr >> 8;
	data[4] = port;
	data[5] = typ;
	data[6] = v1 & 0x08;
	data[7] = v1 >> 8;
	data[8] = v2 & 0x08;
	data[9] = v2 >> 8;
	EthSend(0, 0x0E, LAN_CAN_DETECTOR, data, false, Z21bcCANDetector_s);  //CAN_DETECTOR
}

//--------------------------------------------------------------------------------------------
//Return the state of accessory
void z21Class::setTrntInfo(uint16_t Adr, bool State) {
	byte data[4];
	data[0] = LAN_X_TURNOUT_INFO;  //0x43 X-HEADER
	data[1] = Adr >> 8;   //High
	data[2] = Adr & 0xFF; //Low
	data[3] = State + 1;
	//  if (State == true)
	//    data[3] = 2;
	//  else data[3] = 1;  
	EthSend(0, 0x09, LAN_X_Header, data, true, Z21bcAll_s);
}

//--------------------------------------------------------------------------------------------
//Return CV Value for Programming
void z21Class::setCVReturn (uint16_t CV, uint8_t value) {
	byte data[5];
	data[0] = LAN_X_CV_RESULT;   //0x64 X-Header
	data[1] = 0x14; //DB0
	data[2] = CV >> 8;  //CV_MSB;
	data[3] = CV & 0xFF; //CV_LSB;
	data[4] = value;
	EthSend (0, 0x0A, LAN_X_Header, data, true, Z21bcAll_s);
}

//--------------------------------------------------------------------------------------------
//Return no ACK from Decoder
void z21Class::setCVNack() {
	byte data[2];
	data[0] = LAN_X_CV_NACK;  //0x61 X-Header
	data[1] = 0x13; //DB0
	EthSend (0, 0x07, LAN_X_Header, data, true, Z21bcAll_s);
}

//--------------------------------------------------------------------------------------------
//Return Short while Programming
void z21Class::setCVNackSC() {
	byte data[2];
	data[0] = LAN_X_CV_NACK_SC;   //0x61 X-Header
	data[1] = 0x12; //DB0
	EthSend (0, 0x07, LAN_X_Header, data, true, Z21bcAll_s);
}

//--------------------------------------------------------------------------------------------
//Send Changing of SystemInfo
void z21Class::sendSystemInfo(byte client, uint16_t maincurrent, uint16_t mainvoltage, uint16_t temp) {
	byte data[16];
	data[0] = maincurrent & 0xFF;  //MainCurrent mA
	data[1] = maincurrent >> 8;  //MainCurrent mA
	data[2] = data[0];  //ProgCurrent mA
	data[3] = data[1];  //ProgCurrent mA        
	data[4] = data[0];  //FilteredMainCurrent
	data[5] = data[1];  //FilteredMainCurrent
	data[6] = temp & 0xFF;  //Temperature
	data[7] = temp >> 8;  //Temperature
	data[8] = mainvoltage & 0xFF;  //SupplyVoltage
	data[9] = mainvoltage >> 8;  //SupplyVoltage
	data[10] = data[8];  //VCCVoltage
	data[11] = data[9];  //VCCVoltage
	data[12] = Railpower;  //CentralState
	if (data[12] == csServiceMode)
		data[12] = 0x20;
/*Bitmasken f�r CentralState: 
	#define csEmergencyStop  0x01 // Der Nothalt ist eingeschaltet 
	#define csTrackVoltageOff  0x02 // Die Gleisspannung ist abgeschaltet 
	#define csShortCircuit  0x04 // Kurzschluss 
	#define csProgrammingModeActive 0x20 // Der Programmiermodus ist aktiv 	
*/	
	data[13] = 0x00;  //CentralStateEx
/* Bitmasken f�r CentralStateEx: 
	#define cseHighTemperature  0x01 // zu hohe Temperatur 
	#define csePowerLost  0x02 // zu geringe Eingangsspannung 
	#define cseShortCircuitExternal 0x04 // am externen Booster-Ausgang 
	#define cseShortCircuitInternal 0x08 // am Hauptgleis oder Programmiergleis 	
*/	
	data[14] = 0x00;  //reserved
	data[15] = 0x00;  //reserved
	if (client > 0)		  
		EthSend (client, 0x14, LAN_SYSTEMSTATE_DATACHANGED, data, false, Z21bcNone);	//only to the request client
	EthSend (0, 0x14, LAN_SYSTEMSTATE_DATACHANGED, data, false, Z21bcSystemInfo_s);	//all that select this message (Abo)
}			  

// Private Methods ///////////////////////////////////////////////////////////////////////////////////////////////////
// Functions only available to other functions in this library *******************************************************

//--------------------------------------------------------------------------------------------
void z21Class::EthSend (byte client, unsigned int DataLen, unsigned int Header, byte *dataString, boolean withXOR, byte BC) {
	byte data[24]; 			//z21 send storage
	
	//--------------------------------------------        
	//XOR bestimmen:
	data[0] = DataLen & 0xFF;
	data[1] = DataLen >> 8;
	data[2] = Header & 0xFF;
	data[3] = Header >> 8;
	data[DataLen - 1] = 0;	//XOR

    for (byte i = 0; i < (DataLen-5+!withXOR); i++) { //Ohne Length und Header und XOR
        if (withXOR)
			data[DataLen-1] = data[DataLen-1] ^ *dataString;
		data[i+4] = *dataString;
        dataString++;
    }
   //--------------------------------------------        		
   if (client > 0) {
		if (notifyz21EthSend)
			notifyz21EthSend(client, data);

		#if defined (SERIALDEBUG)
			  ZDebug.print("CTX ");
			  ZDebug.print(client);
			  ZDebug.print(" : ");
			  for (byte x = 0; x < data[0]; x++) {
				  ZDebug.print(data[x], HEX);
				  ZDebug.print(" ");
			  }
			  ZDebug.println();
		#endif
   }
   else {
	byte clientOut = client;
	for (byte i = 0; i < z21clientMAX; i++) {
		if ( (ActIP[i].time > 0) && ( (BC & ActIP[i].BCFlag) > 0) ) {    //Boradcast & Noch aktiv

		  if (BC != 0) {
			if (BC == Z21bcAll_s)
				clientOut = 0;	//ALL
			else clientOut = ActIP[i].client;
		  }
		  
		  //--------------------------------------------
		  if (notifyz21EthSend)
			notifyz21EthSend(clientOut, data);

		  #if defined (SERIALDEBUG)
			  ZDebug.print(i);
			  ZDebug.print("BTX ");
			  ZDebug.print(clientOut);
			  ZDebug.print(" BC:");
			  ZDebug.print(BC & ActIP[i].BCFlag, BIN);
			  ZDebug.print(" : ");
			  for (byte x = 0; x < data[0]; x++) {
				  ZDebug.print(data[x], HEX);
				  ZDebug.print(" ");
			  }
			  ZDebug.println();
		  #endif
		  if (clientOut == 0)
			  return;
		}
	}
  }
}

//--------------------------------------------------------------------------------------------
//Convert local stored flag back into a Z21 Flag
unsigned long z21Class::getz21BcFlag (byte flag) {
  unsigned long outFlag = 0;
  if ((flag & Z21bcAll_s) != 0)
    outFlag |= Z21bcAll;
  if ((flag & Z21bcRBus_s) != 0)
    outFlag |= Z21bcRBus;
  if ((flag & Z21bcSystemInfo_s) != 0)
    outFlag |= Z21bcSystemInfo;
  if ((flag & Z21bcNetAll_s) != 0)
    outFlag |= Z21bcNetAll;
  if ((flag & Z21bcLocoNet_s) != 0)
    outFlag |= Z21bcLocoNet;
  if ((flag & Z21bcLocoNetLocos_s) != 0)
    outFlag |= Z21bcLocoNetLocos;
  if ((flag & Z21bcLocoNetSwitches_s) != 0)
    outFlag |= Z21bcLocoNetSwitches;
  if ((flag & Z21bcLocoNetGBM_s) != 0)    
    outFlag |= Z21bcLocoNetGBM;
  return outFlag;
}

//--------------------------------------------------------------------------------------------
//Convert Z21 LAN BC flag to local stored flag
byte z21Class::getLocalBcFlag (unsigned long flag) {
  byte outFlag = 0;
  if ((flag & Z21bcAll) != 0)
    outFlag |= Z21bcAll_s;
  if ((flag & Z21bcRBus) != 0) 
    outFlag |= Z21bcRBus_s;
  if ((flag & Z21bcSystemInfo) != 0)
    outFlag |= Z21bcSystemInfo_s;
  if ((flag & Z21bcNetAll) != 0)
    outFlag |= Z21bcNetAll_s;
  if ((flag & Z21bcLocoNet) != 0)
    outFlag |= Z21bcLocoNet_s;
  if ((flag & Z21bcLocoNetLocos) != 0)
    outFlag |= Z21bcLocoNetLocos_s;
  if ((flag & Z21bcLocoNetSwitches) != 0)
    outFlag |= Z21bcLocoNetSwitches_s;
  if ((flag & Z21bcLocoNetGBM) != 0) 
    outFlag |= Z21bcLocoNetGBM_s;
  return outFlag;  
}

//--------------------------------------------------------------------------------------------
// delete the stored IP-Address
void z21Class::clearIP (byte pos) {
			ActIP[pos].client = 0;
			ActIP[pos].BCFlag = 0;
			ActIP[pos].time = 0;
			ActIP[pos].adr = 0;
}

//--------------------------------------------------------------------------------------------
void z21Class::clearIPSlots() {
  for (int i = 0; i < z21clientMAX; i++) 
    clearIP(i);
}

//--------------------------------------------------------------------------------------------
void z21Class::clearIPSlot(byte client) {
  for (int i = 0; i < z21clientMAX; i++) {
	  if (ActIP[i].client == client) {
		  clearIP(i);
		  return;
	  }
  }
}

//--------------------------------------------------------------------------------------------
byte z21Class::addIPToSlot (byte client, byte BCFlag) {
  byte Slot = z21clientMAX;
  for (byte i = 0; i < z21clientMAX; i++) {
    if (ActIP[i].client == client) {
      ActIP[i].time = z21ActTimeIP;
      if (BCFlag != 0)    //Falls BC Flag �bertragen wurde diesen hinzuf�gen!
        ActIP[i].BCFlag = BCFlag;
      return ActIP[i].BCFlag;    //BC Flag 4. Byte R�ckmelden
    }
    else if (ActIP[i].time == 0 && Slot == z21clientMAX)
      Slot = i;
  }
  ActIP[Slot].client = client;
  ActIP[Slot].time = z21ActTimeIP;
  setPower(Railpower);
  return ActIP[Slot].BCFlag;   //BC Flag 4. Byte R�ckmelden
}

//--------------------------------------------------------------------------------------------
//check if there are slots with the same loco, set them to busy
void z21Class::setOtherSlotBusy(byte slot) {
	for (byte i = 0; i < z21clientMAX; i++) {
		if ((i != slot) && (ActIP[slot].adr == ActIP[i].adr)) { //if in other Slot -> set busy
			ActIP[i].adr = 0; //clean slot that informed as busy & let it activ
			//Inform with busy message:
			//not used!
		}
	}
}

//--------------------------------------------------------------------------------------------
//Add loco to slot. 
void z21Class::addBusySlot (byte client, uint16_t adr) {
	for (byte i = 0; i < z21clientMAX; i++) {
		if (ActIP[i].client == client) {
			if (ActIP[i].adr != adr) {	//skip is already used by this client
				ActIP[i].adr = adr;		//store loco that is used
				setOtherSlotBusy(i);	//make other busy
			}
			break;
		}
	}
}

//--------------------------------------------------------------------------------------------
//used by non Z21 client
void z21Class::reqLocoBusy (uint16_t adr) {
	for (byte i = 0; i < z21clientMAX; i++) {
		if (adr == ActIP[i].adr) {
			ActIP[i].adr = 0;	//clear
		}
	}
}
