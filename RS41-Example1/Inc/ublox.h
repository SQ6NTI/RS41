/*
 * ublox.h
 *
 *  Created on: Oct 28, 2017
 *      Author: Milosz Iskrzynski SQ6NTI
 */

#ifndef UBLOX_H_
#define UBLOX_H_

#include <stdint.h>
#include "stm32f1xx_hal.h"

/* UBX sync bytes */
#define UBX_SYNC1 0xB5
#define UBX_SYNC2 0x62

/* UBX NAV Message Class */
#define UBX_NAV 0x01
#define UBX_NAV_POSLLH 0x02
#define UBX_NAV_STATUS 0x03
#define UBX_NAV_SOL 0x06
#define UBX_NAV_TIMEUTC 0x21

/* UBX RXM Message Class */
#define UBX_RXM 0x02

/* UBX INF Message Class */
#define UBX_INF 0x04

/* UBX ACK Message Class */
#define UBX_ACK 0x05
#define UBX_ACK_NAK 0x00
#define UBX_ACK_ACK 0x01

/* UBX CFG Message Class */
#define UBX_CFG 0x06
#define UBX_CFG_PRT 0x00
#define UBX_CFG_MSG 0x01
#define UBX_CFG_RST 0x04
#define UBX_CFG_RXM 0x11
#define UBX_CFG_NAV5 0x24

/* Other UBX Message Classes */
#define UBX_MON 0x0A
#define UBX_AID 0x0B
#define UBX_TIM 0x0D
#define UBX_ESF 0x10

/* NMEA Standard Message Class */
#define NMEA_STD 0xF0
#define NMEA_STD_GGA 0x00
#define NMEA_STD_GLL 0x01
#define NMEA_STD_GSA 0x02
#define NMEA_STD_GSV 0x03
#define NMEA_STD_RMC 0x04
#define NMEA_STD_VTG 0x05

/* NMEA Proprietary Message Class */
#define NMEA_PROP 0xF0

/* UBX header */
typedef struct  __attribute__((packed)) {
	uint8_t syncChar1;			// 0xB5
	uint8_t syncChar2;			// 0x62
	uint8_t messageClass;
	uint8_t messageId;
	uint16_t payloadLength;
} ubxHeader;

/* UBX ACK-ACK message (0x05 0x01) */
typedef struct {
	uint8_t msgClass;
	uint8_t msgID;
} ubxAckAck;

/* UBX ACK-NAK message (0x05 0x00) */
typedef struct {
	uint8_t msgClass;
	uint8_t msgID;
} ubxAckNak;

/* UBX CFG-MSG message (0x06 0x01) */
typedef struct {
	uint8_t msgClass;
	uint8_t msgID;
	uint8_t rate;
} ubxCfgMsg;

/* UBX CFG-NAV5 message (0x06 0x24) */
typedef struct {
	uint16_t mask;				// Parameters Bitmask 0b0000000011111111 to set all params)
	uint8_t dynModel;			// Dynamic Platform model: 0 Portable, 2 Stationary, 3 Pedestrian, 4 Automotive, 5 Sea, 6 Airborne with <1g Acceleration, 7 Airborne with <2g Acceleration, 8 Airborne with <4g Acceleration
	uint8_t fixMode;			// Position Fixing Mode. - 1: 2D only - 2: 3D only - 3: Auto 2D/3D
	int32_t fixedAlt;			// Fixed altitude (mean sea level) for 2D fix mode. [0.01 m]
	uint32_t fixedAltVar;		// Fixed altitude variance for 2D mode. [0.0001 m^2]
	int8_t minElev;				// Minimum Elevation for a GNSS satellite to be used in NAV [deg]
	uint8_t drLimit;			// Maximum time to perform dead reckoning (linear extrapolation) in case of GPS signal loss [s]
	uint16_t pDop;				// Position DOP Mask to use [0.1]
	uint16_t tDop;				// Time DOP Mask to use [0.1]
	uint16_t pAcc;				// Position Accuracy Mask [m]
	uint16_t tAcc;				// Time Accuracy Mask [m]
	uint8_t staticHoldThresh;	// Static hold threshold [cm/s]
	uint8_t dgpsTimeOut;		// DGPS timeout, firmware 7 and newer only [s]
	uint32_t reserved2;			// Always set to zero
	uint32_t reserved3;			// Always set to zero
	uint32_t reserved4;			// Always set to zero
} ubxCfgNav5;

/* UBX CFG-PRT message (0x06 0x00) */
typedef struct {
	uint8_t portID;				// Port Identifier Number (= 1 or 2 for UART ports)
	uint8_t reserved1;			// Reserved
	uint16_t txReady;			// TX ready PIN configuration
	uint32_t mode;				// A bit mask describing the UART mode
	uint32_t baudRate;			// Baud rate in bits/second [Bits/s]
	uint16_t inProtoMask;		// A mask describing which input protocols are active
	uint16_t outProtoMask;		// A mask describing which output protocols are active
	uint8_t reserved2[2];		// Reserved
} ubxCfgPrt;

/* UBX CFG-RST message (0x06 0x04) */
typedef struct {
	uint16_t navBbrMask;		// BBR Sections to clear. (Special: 0x0000 Hotstart, 0x0001 Warmstart, 0xFFFF Coldstart)
	uint8_t resetMode;			// Reset Type
	uint8_t reserved1;			// Reserved
} ubxCfgRst;

/* UBX CFG-RXM message (0x06 0x11) */
typedef struct {
	uint8_t reserved1;			// Always set to 8
	uint8_t lpMode;				// Low Power Mode (0: max performance, 1: power save, 4: eco)
} ubxCfgRxm;

/* UBX NAV-POSLLH message (0x01 0x02) */
typedef struct {
	uint32_t iTOW;				// GPS Millisecond Time of Week [ms]
	int32_t lon;				// Longitude [1e-7 deg]
	int32_t lat;				// Latitude [1e-7 deg]
	int32_t height;				// Height above Ellipsoid [mm]
	int32_t hMSL;				// Height above mean sea level [mm]
	uint32_t hAcc;				// Horizontal Accuracy Estimate [mm]
	uint32_t vAcc;				// Vertical Accuracy Estimate [mm]
} ubxNavPosllh;

/* UBX NAV-SOL message (0x01 0x06) */
typedef struct {
	uint32_t iTOW;				// GPS Millisecond Time of Week [ms]
	int32_t fTOW;				// Fractional Nanoseconds remainder of rounded ms above, range -500000 .. 500000 [ns]
	int16_t week;				// GPS week (GPS time)
	uint8_t gpsFix;				// GPSfix Type (0x00 No Fix, 01 Dead Reckoning only, 02 2D-Fix, 03 3D-Fix, 04 GPS + dead reckoning combined, 05 Time only fix)
	uint8_t flags;				// Fix Status Flags
	int32_t ecefX;				// ECEF X coordinate [cm]
	int32_t ecefY;				// ECEF Y coordinate [cm]
	int32_t ecefZ;				// ECEF Z coordinate [cm]
	uint32_t pAcc;				// 3D Position Accuracy Estimate [cm]
	int32_t ecefVX;				// ECEF X velocity [cm/s]
	int32_t ecefVY;				// ECEF Y velocity [cm/s]
	int32_t ecefVZ;				// ECEF Z velocity [cm/s]
	uint32_t sAcc;				// Speed Accuracy Estimate [cm/s]
	uint16_t pDOP;				// Position DOP [0.01]
	uint8_t reserved1;			// Reserved
	uint8_t numSV;				// Number of SVs used in Nav Solution
	uint32_t reserved2;			// Reserved
} ubxNavSol;

/* UBX NAV-STATUS message (0x01 0x03) */
typedef struct {
	uint32_t iTOW;				// GPS Millisecond Time of Week [ms]
	uint8_t gpsFix;				// GPSfix Type (0x00 No Fix, 01 Dead Reckoning only, 02 2D-Fix, 03 3D-Fix, 04 GPS + dead reckoning combined, 05 Time only fix)
	uint8_t flags;				// Fix Status Flags
	uint8_t fixStat;			// Fix Status Information
	uint8_t flags2;				// Further information about navigation output
	uint32_t ttff;				// Time to first fix (millisecond time tag)
	uint32_t msss;				// Milliseconds since Startup / Reset
} ubxNavStatus;

/* UBX NAV-TIMEUTC message (0x01 0x021) */
typedef struct {
	uint32_t iTOW;			// GPS Millisecond Time of Week [ms]
	uint32_t tAcc;			// Time Accuracy Estimate [ns]
	int32_t nano;			// Nanoseconds of second, range -1e9 .. 1e9 (UTC) [ns]
	uint16_t year;			// Year, range 1999..2099 (UTC) [y]
	uint8_t month;			// Month, range 1..12 (UTC) [month]
	uint8_t day;			// Day of Month, range 1..31 (UTC) [d]
	uint8_t hour;			// Hour of Day, range 0..23 (UTC) [h]
	uint8_t min;			// Minute of Hour, range 0..59 (UTC) [min]
	uint8_t sec;			// Seconds of Minute, range 0..59 (UTC) [s]
	uint8_t valid;			// Validity Flags
} ubxNavTimeutc;

/* UBX Message */
typedef union {
	ubxAckAck		ackack;
	ubxAckNak		acknak;
	ubxCfgMsg		cfgmsg;
	ubxCfgNav5		cfgnav5;
	ubxCfgPrt		cfgprt;
	ubxCfgRst		cfgrst;
	ubxCfgRxm		cfgrxm;
	ubxNavPosllh	navposllh;
	ubxNavStatus	navstatus;
	ubxNavSol		navsol;
	ubxNavTimeutc	navtimeutc;
} ubxMessage;

/* UBX checksum */
typedef struct {
	uint8_t ck_a;
	uint8_t ck_b;
} ubxChecksum;

/* UBX packet */
typedef struct __attribute__((packed)) {
	ubxHeader header;
	ubxMessage message;
	ubxChecksum checksum;
} ubxPacket;

/* Ublox GPS current data */
typedef struct {
	int32_t lat;
	int32_t lon;
	int32_t alt;
	int32_t speed;
	uint8_t sats;
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
	uint8_t fix;
} ubxGPSData;

void ubxInit(UART_HandleTypeDef *uart);
void ubxRxByte(uint8_t data);
void ubxProcessPacket(const ubxPacket *packet);
int ubxResponseWait(int timeout);
void ubxSendPacket(uint8_t messageClass, uint8_t messageId, uint16_t payloadLength, ubxMessage message);
void ubxTxPacket(ubxPacket packet);
uint8_t *serializeUbxPacket(ubxPacket *ubxPacket);
ubxChecksum ubxCalcChecksum(const ubxPacket *packet);
ubxGPSData ubxLastGPSData(void);

extern void UART_ReInit(UART_HandleTypeDef*, uint32_t);
extern void UART_TxStart(UART_HandleTypeDef*);
extern int UART_TxFinished(UART_HandleTypeDef*);

#endif /* UBLOX_H_ */
