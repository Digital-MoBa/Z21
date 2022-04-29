/*
 * Z21type.h
 *  Created on: 16.04.2016
 *      Author: Philipp Gahtow
 *
 *	19.07.16 Add X-Bus (XpressNet)
 *  19.06.17 Add CAN and new BC-Flags
 *
*/

//**************************************************************
//Z21 LAN Protokoll Spezifikation:
#define LAN_X_Header                 0x40  //not in Spezifikation!
#define LAN_GET_SERIAL_NUMBER        0x10
#define LAN_GET_CODE				 0x18	//SW Feature-Umfang der Z21 
#define LAN_LOGOFF                   0x30
#define LAN_X_GET_SETTING            0x21  
#define LAN_X_BC_TRACK_POWER         0x61
#define LAN_X_UNKNOWN_COMMAND        0x61
#define LAN_X_STATUS_CHANGED         0x62
#define LAN_X_GET_VERSION			 0x63	//AW: X-Bus Version 090040006321301260
#define LAN_X_SET_STOP               0x80  //AW: LAN_X_BC_STOPPED
#define LAN_X_BC_STOPPED             0x81
#define LAN_X_GET_FIRMWARE_VERSION   0xF1  //AW: 0xF3
#define LAN_SET_BROADCASTFLAGS       0x50
#define LAN_GET_BROADCASTFLAGS       0x51
#define LAN_SYSTEMSTATE_DATACHANGED  0x84
#define LAN_SYSTEMSTATE_GETDATA      0x85  //AW: LAN_SYSTEMSTATE_DATACHANGED
#define LAN_GET_HWINFO               0x1A
#define LAN_GET_LOCOMODE             0x60
#define LAN_SET_LOCOMODE             0x61
#define LAN_GET_TURNOUTMODE          0x70
#define LAN_SET_TURNOUTMODE          0x71
#define LAN_X_GET_LOCO_INFO          0xE3
#define LAN_X_SET_LOCO               0xE4  //X-Header
#define LAN_X_SET_LOCO_FUNCTION      0xF8  //DB0
#define LAN_X_SET_LOCO_BINARY_STATE  0xE5  //X-Header
#define LAN_X_LOCO_INFO              0xEF
#define LAN_X_GET_TURNOUT_INFO       0x43 
#define LAN_X_SET_TURNOUT            0x53
#define LAN_X_TURNOUT_INFO           0x43 
#define LAN_X_SET_EXT_ACCESSORY		 0x54	//new: 1.10
#define LAN_X_GET_EXT_ACCESSORY_INFO 0x44	//new: 1.10
#define LAN_X_CV_READ                0x23
#define LAN_X_CV_WRITE               0x24
#define LAN_X_CV_NACK_SC             0x61
#define LAN_X_CV_NACK                0x61
#define LAN_X_CV_RESULT              0x64
#define LAN_RMBUS_DATACHANGED        0x80
#define LAN_RMBUS_GETDATA            0x81
#define LAN_RMBUS_PROGRAMMODULE      0x82

#define LAN_RAILCOM_DATACHANGED      0x88
#define LAN_RAILCOM_GETDATA          0x89

#define LAN_LOCONET_Z21_RX           0xA0
#define LAN_LOCONET_Z21_TX           0xA1
#define LAN_LOCONET_FROM_LAN         0xA2
#define LAN_LOCONET_DISPATCH_ADDR    0xA3
#define LAN_LOCONET_DETECTOR         0xA4

#define LAN_CAN_DETECTOR 			 0xC4

#define LAN_X_CV_POM                 0xE6  //X-Header 
#define LAN_X_CV_POM_WRITE_BYTE		 0xEC  //DB3 Option
#define LAN_X_CV_POM_WRITE_BIT		 0xE8  //DB3 Option
#define LAN_X_CV_POM_READ_BYTE		 0xE4  //DB3 Option
#define LAN_X_CV_POM_ACCESSORY_WRITE_BYTE	0xEC	//DB3 Option
#define LAN_X_CV_POM_ACCESSORY_WRITE_BIT	0xE8	//DB3 Option
#define LAN_X_CV_POM_ACCESSORY_READ_BYTE	0xE4	//DB3 Option

//ab Z21 FW Version 1.23
#define LAN_X_MM_WRITE_BYTE          0x24

//ab Z21 FW Version 1.25
#define LAN_X_DCC_READ_REGISTER      0x22
#define LAN_X_DCC_WRITE_REGISTER     0x23

//**************************************************************
//Z21 BC Flags
#define Z21bcNone                B00000000
#define Z21bcAll    		0x00000001
#define Z21bcAll_s               B00000001
#define Z21bcRBus   		0x00000002
#define Z21bcRBus_s              B00000010
#define Z21bcRailcom    	0x00000004    //RailCom-Daten für Abo Loks
#define Z21bcRailcom_s			0x100

#define Z21bcSystemInfo 	0x00000100	//LAN_SYSTEMSTATE_DATACHANGED
#define Z21bcSystemInfo_s        B00000100

//ab FW Version 1.20:
#define Z21bcNetAll          0x00010000 // Alles, auch alle Loks ohne vorher die Lokadresse abonnieren zu müssen (für PC Steuerung)
#define Z21bcNetAll_s            B00001000

#define Z21bcLocoNet         0x01000000 // LocoNet Meldungen an LAN Client weiterleiten (ohne Loks und Weichen)
#define Z21bcLocoNet_s           B00010000
#define Z21bcLocoNetLocos    0x02000000 // Lok-spezifische LocoNet Meldungen an LAN Client weiterleiten
#define Z21bcLocoNetLocos_s      B00110000
#define Z21bcLocoNetSwitches 0x04000000 // Weichen-spezifische LocoNet Meldungen an LAN Client weiterleiten
#define Z21bcLocoNetSwitches_s   B01010000

//ab FW Version 1.22:
#define Z21bcLocoNetGBM      0x08000000  //Status-Meldungen von Gleisbesetztmeldern am LocoNet-Bus
#define Z21bcLocoNetGBM_s        B10010000

//ab FW Version 1.29:
#define Z21bcRailComAll		 0x00040000 //alles: Änderungen bei RailCom-Daten ohne Lok Abo! -> LAN_RAILCOM_DATACHANGED
#define Z21bcRailComAll_s		 B10000000

//ab FW Version 1.30:
#define Z21bcCANDetector	 0x00080000	//Meldungen vom Gelisbesetztmeldern am CAN-Bus
#define Z21bcCANDetector_s		 B11000000