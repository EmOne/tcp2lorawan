//------------------------------------------------------------------------------
//
//	File:		WiMOD_LoRaWAN_API.h
//
//	Abstract:	API Layer of LoRaWAN Host Controller Interface
//
//	Version:	0.1
//
//	Date:		18.05.2016
//
//	Disclaimer:	This example code is provided by IMST GmbH on an "AS IS"
//              basis without any warranties.
//
//	Maintain by : Anol Paisal <anol.paisal@emone.co.th>
//------------------------------------------------------------------------------

#ifndef WIMOD_LORAWAN_API_H
#define WIMOD_LORAWAN_API_H

//------------------------------------------------------------------------------
//
//  Include Files
//
//------------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>

#include <main.h>
//------------------------------------------------------------------------------
//
//  General Declarations
//
//------------------------------------------------------------------------------

typedef uint8_t     UINT8;
typedef uint16_t    UINT16;
typedef uint32_t    UINT32;


// helper struct for ID -> string conversion
typedef struct
{
    int         ID;
    const char* String;
}TIDString;

//------------------------------------------------------------------------------
//
//  Endpoint (SAP) Identifier
//
//------------------------------------------------------------------------------

#define DEVMGMT_SAP_ID                      0x01
#define LORAWAN_SAP_ID                      0x10

//------------------------------------------------------------------------------
//
//  Device Management SAP Message Identifier
//
//------------------------------------------------------------------------------

// Message Identifier
#define DEVMGMT_MSG_PING_REQ                    0x01
#define DEVMGMT_MSG_PING_RSP                    0x02

#define DEVMGMT_MSG_GET_DEVICE_INFO_REQ         0x03
#define DEVMGMT_MSG_GET_DEVICE_INFO_RSP         0x04

#define DEVMGMT_MSG_GET_FW_VERSION_REQ          0x05
#define DEVMGMT_MSG_GET_FW_VERSION_RSP          0x06

#define DEVMGMT_MSG_RESET_REQ          			0x07
#define DEVMGMT_MSG_RESET_RSP          			0x08

#define DEVMGMT_MSG_SET_OPMODE_REQ 				0x09
#define DEVMGMT_MSG_SET_OPMODE_RSP 				0x0A
#define DEVMGMT_MSG_GET_OPMODE_REQ 				0x0B
#define DEVMGMT_MSG_GET_OPMODE_RSP 				0x0C

#define DEVMGMT_MSG_SET_RTC_REQ 				0x0D
#define DEVMGMT_MSG_SET_RTC_RSP 				0x0E
#define DEVMGMT_MSG_GET_RTC_REQ 				0x0F
#define DEVMGMT_MSG_GET_RTC_RSP 				0x10

#define DEVMGMT_MSG_GET_DEVICE_STATUS_REQ 		0x17
#define DEVMGMT_MSG_GET_DEVICE_STATUS_RSP 		0x18

#define DEVMGMT_MSG_POWER_UP_IND				0x20

#define DEVMGMT_MSG_SET_RTC_ALARM_REQ 			0x31
#define DEVMGMT_MSG_SET_RTC_ALARM_RSP 			0x32
#define DEVMGMT_MSG_CLEAR_RTC_ALARM_REQ 		0x33
#define DEVMGMT_MSG_CLEAR_RTC_ALARM_RSP 		0x34
#define DEVMGMT_MSG_GET_RTC_ALARM_REQ			0x35
#define DEVMGMT_MSG_GET_RTC_ALARM_RSP 			0x36
#define DEVMGMT_MSG_RTC_ALARM_IND 				0x38

#define DEVMGMT_MSG_SET_HCI_CFG_REQ 			0x41
#define DEVMGMT_MSG_SET_HCI_CFG_RSP 			0x42
#define DEVMGMT_MSG_GET_HCI_CFG_REQ 			0x43
#define DEVMGMT_MSG_GET_HCI_CFG_RSP 			0x44

// Status Identifier
#define	DEVMGMT_STATUS_OK                       0x00	//Operation successful
#define	DEVMGMT_STATUS_ERROR                    0x01	//Operation failed
#define	DEVMGMT_STATUS_CMD_NOT_SUPPORTED        0x02	//Command is not supported
#define	DEVMGMT_STATUS_WRONG_PARAMETER          0x03	//HCI message contains wrong parameter
#define DEVMGMT_STATUS_WRONG_DEVICE_MODE        0x04	//Stack is running in a wrong mode

//------------------------------------------------------------------------------
//
//  LoRaWAN SAP Message Identifier
//
//------------------------------------------------------------------------------

// Message Identifier
#define LORAWAN_MSG_ACTIVATE_DEVICE_REQ			0x01
#define LORAWAN_MSG_ACTIVATE_DEVICE_RSP			0x02
#define LORAWAN_MSG_SET_JOIN_PARAM_REQ 			0x05
#define LORAWAN_MSG_SET_JOIN_PARAM_RSP 			0x06

#define LORAWAN_MSG_JOIN_NETWORK_REQ            0x09
#define LORAWAN_MSG_JOIN_NETWORK_RSP            0x0A
#define LORAWAN_MSG_JOIN_TRANSMIT_IND           0x0B
#define LORAWAN_MSG_JOIN_NETWORK_IND            0x0C

#define LORAWAN_MSG_SEND_UDATA_REQ              0x0D
#define LORAWAN_MSG_SEND_UDATA_RSP              0x0E
#define LORAWAN_MSG_SEND_UDATA_IND              0x0F
#define LORAWAN_MSG_RECV_UDATA_IND              0x10

#define LORAWAN_MSG_SEND_CDATA_REQ              0x11
#define LORAWAN_MSG_SEND_CDATA_RSP              0x12
#define LORAWAN_MSG_SEND_CDATA_IND              0x13
#define LORAWAN_MSG_RECV_CDATA_IND              0x14

#define LORAWAN_MSG_RECV_ACK_IND                0x15
#define LORAWAN_MSG_RECV_NODATA_IND             0x16

#define LORAWAN_MSG_SET_RSTACK_CONFIG_REQ 		0x19
#define LORAWAN_MSG_SET_RSTACK_CONFIG_RSP 		0x1A
#define LORAWAN_MSG_GET_RSTACK_CONFIG_REQ 		0x1B
#define LORAWAN_MSG_GET_RSTACK_CONFIG_RSP 		0x1C

#define LORAWAN_MSG_REACTIVATE_DEVICE_REQ 		0x1D
#define LORAWAN_MSG_REACTIVATE_DEVICE_RSP 		0x1E

#define LORAWAN_MSG_DEACTIVATE_DEVICE_REQ 		0x21
#define LORAWAN_MSG_DEACTIVATE_DEVICE_RSP 		0x22

#define LORAWAN_MSG_FACTORY_RESET_REQ 			0x23
#define LORAWAN_MSG_FACTORY_RESET_RSP 			0x24

#define LORAWAN_MSG_SET_DEVICE_EUI_REQ 			0x25
#define LORAWAN_MSG_SET_DEVICE_EUI_RSP 			0x26
#define LORAWAN_MSG_GET_DEVICE_EUI_REQ 			0x27
#define LORAWAN_MSG_GET_DEVICE_EUI_RSP 			0x28

#define LORAWAN_MSG_GET_NWK_STATUS_REQ 			0x29
#define LORAWAN_MSG_GET_NWK_STATUS_RSP 			0x2A

#define LORAWAN_MSG_SEND_MAC_CMD_REQ 			0x2B
#define LORAWAN_MSG_SEND_MAC_CMD_RSP 			0x2C
#define LORAWAN_MSG_RECV_MAC_CMD_IND 			0x2D

#define LORAWAN_MSG_SET_BATTERY_LEVEL_REQ		0x2E
#define LORAWAN_MSG_SET_BATTERY_LEVEL_RSP		0x2F

#define LORAWAN_MSG_SET_CUSTOM_CFG_REQ 			0x31
#define LORAWAN_MSG_SET_CUSTOM_CFG_RSP 			0x32
#define LORAWAN_MSG_GET_CUSTOM_CFG_REQ 			0x33
#define LORAWAN_MSG_GET_CUSTOM_CFG_RSP 			0x34

#define LORAWAN_MSG_GET_SUPPORTED_BANDS_REQ 	0x35
#define LORAWAN_MSG_GET_SUPPORTED_BANDS_RSP 	0x36

#define LORAWAN_MSG_SET_LINKADRREQ_CONFIG_REQ 	0x3B
#define LORAWAN_MSG_SET_LINKADRREQ_CONFIG_RSP 	0x3C
#define LORAWAN_MSG_GET_LINKADRREQ_CONFIG_REQ 	0x3D
#define LORAWAN_MSG_GET_LINKADRREQ_CONFIG_RSP 	0x3E

// Status Identifier
#define LORAWAN_STATUS_OK                       0x00	//Operation successful
#define	LORAWAN_STATUS_ERROR                    0x01	//Operation failed
#define	LORAWAN_STATUS_CMD_NOT_SUPPORTED        0x02	//Command is not supported
#define	LORAWAN_STATUS_WRONG_PARAMETER          0x03	//HCI message contains wrong parameter
#define	LORAWAN_STATUS_WRONG_DEVICE_MODE 		0x04	//Stack is running in a wrong mode
#define	LORAWAN_STATUS_DEVICE_NOT_ACTIVATED 	0x05	//Device is not activated
#define	LORAWAN_STATUS_DEVICE_BUSY				0x06	//Device is busy, command rejected
#define	LORAWAN_STATUS_QUEUE_FULL				0x07	//Message queue is full, command rejected
#define	LORAWAN_STATUS_LENGTH_ERROR				0x08	//HCI message length is invalid or radio payload size is too large
#define	LORAWAN_STATUS_NO_FACTORY_SETTINGS		0x09	//Factory Settings EEPROM block missing
#define	LORAWAN_STATUS_CHANNEL_BLOCKED_BY_DC	0x0A	//Channel blocked by Duty Cycle
#define LORAWAN_STATUS_CHANNEL_NOT_AVAILABLE 	0x0B 	//No channel available (e.g. no channel defined for the configured spreading factor)
//------------------------------------------------------------------------------
//
//  Function Prototypes
//
//------------------------------------------------------------------------------

// init
bool
WiMOD_LoRaWAN_Init(
#ifdef Q_OS_WIN
		const char*              comPort        // comPort
#else
		UART_HandleTypeDef * comPort
#endif
		);

int WiMOD_DevMgmt_Msg_Req(uint8_t msg_id, uint8_t* val, uint16_t len);
int WiMOD_LoRaWAN_Msg_Req(uint8_t msg_id, uint8_t* val, uint16_t len);

//Get CustomConfig
int
WiMOD_LoRaWAN_GetCustomConfig(void);

//set CustomConfig
int
WiMOD_LoRaWAN_SetCustomConfig(int8_t val);

//set RStackConfig
int
WiMOD_LoRaWAN_SetRadioStackConfig(void);

//set OPMODE
int
WiMOD_LoRaWAN_SetOPMODE(uint8_t val);
//get OPMODE
int
WiMOD_LoRaWAN_GetOPMODE(void);
// clear RTC alarm
int
WiMOD_LoRaWAN_ClearRTCAlarm(void);
// get RTC alarm
int
WiMOD_LoRaWAN_GetRTCAlarm(void);
// set RTC alarm
int
WiMOD_LoRaWAN_SetRTCAlarm(uint8_t* alarm);
// get RTC
int
WiMOD_LoRaWAN_GetRTC(void);
// set RTC
int
WiMOD_LoRaWAN_SetRTC(uint32_t* time);
// ping device
int
WiMOD_LoRaWAN_SendPing(void);
// get device information
int
WiMOD_LoRaWAN_GetDeviceInfo(void);
// get firmware Version
int
WiMOD_LoRaWAN_GetFirmwareVersion(void);
// reset device
int
WiMOD_LoRaWAN_Reset(void);
// get device status
int
WiMOD_LoRaWAN_GetDeviceStatus(void);

// get device status
int
WiMOD_LoRaWAN_GetNkwStatus(void);
// factory reset
int
WiMOD_LoRaWAN_FactoryReset(void);

// Activate network
int
WiMOD_LoRaWAN_ActivateRequest(void);

// join network
int
WiMOD_LoRaWAN_JoinNetworkRequest(void);

// send unconfirmed radio data
int
WiMOD_LoRaWAN_SendURadioData(UINT8 port, UINT8* data, size_t length);

// send confirmed radio data
int
WiMOD_LoRaWAN_SendCRadioData(UINT8 port, UINT8* data, size_t length);

// get link adr
int
WiMOD_LoRaWAN_GetLinkADR(void);

// set link adr
int
WiMOD_LoRaWAN_SetLinkADR(uint8_t * payload);

// set mac cmd
int
WiMOD_LoRaWAN_SetMAC_CMD(uint8_t * payload);

// receiver process
void
WiMOD_LoRaWAN_Process(void);

// Todo : add your own platform specific variables here
// forward declarations
void Ping(void);
void GetNwkStatus(void);
void GetCustomCFG(void);
void SetCustomCFG(void);
void ActivateABP(void);
void Reactivate(void);
void Deactivate(void);
void SetRadioStack(void);
void GetRadioStack(void);
void GetSupportBand(void);
void GetDevEUI(void);
void SetDevEUI(void);
void GetOPMODE(void);
void SetOPMODE(void);
void ClearOPMODE(void);
void ClearRTCAlarm(void);
void GetRTC(void);
void SetRTC(void);
void GetAlarm(void);
void SetAlarm(void);
void GetFirmwareVersion(void);
void GetDeviceInfo(void);
void GetDeviceStatus(void);
void Reset(void);
void Join(void);
int SendUData(UINT8 port,       // LoRaWAN Port
		UINT8* srcData,    // application payload
		size_t srcLength);
int SendCData(UINT8 port,       // LoRaWAN Port
		UINT8* srcData,    // application payload
		size_t srcLength);
int SetBatLVL(UINT8 batLvl);
void FactoryReset(void);
void GetLinkADR(void);
void SetLinkADR(void);

#endif // WIMOD_LORAWAN_API_H

//------------------------------------------------------------------------------
// end of file
//------------------------------------------------------------------------------
