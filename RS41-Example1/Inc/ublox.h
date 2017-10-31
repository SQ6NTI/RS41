/*
 * ublox.h
 *
 *  Created on: Oct 28, 2017
 *      Author: Milosz Iskrzynski SQ6NTI
 */

#ifndef UBLOX_H_
#define UBLOX_H_

/* UBX sync bytes */
#define UBX_SYNC1 0xB5
#define UBX_SYNC2 0x62

/* UBX NAV Message Class */
#define UBX_NAV 0x01
#define UBX_NAV_POSLLH 0x02
#define UBX_NAV_SOL 0x06
#define UBX_NAV_TIMEUTC 0x21

/* UBX RXM Message Class */
#define UBX_RXM 0x02

/* UBX INF Message Class */
#define UBX_INF 0x04

/* UBX ACK Message Class */
#define UBX_ACK 0x05
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

/* UBX State definition */
typedef enum {
	UBX_STATE_IDLE = 0,
	UBX_STATE_
};

/* UBX header */
typedef struct  __attribute__((packed)) {
	uint8_t syncChar1;	// 0xB5
	uint8_t syncChar2;	// 0x62
	uint8_t messageClass;
	uint8_t messageId;
	uint16_t payloadLength;
} ubxHeader;

/* UBX ACK-ACK message (0x05 0x01) */
typedef struct  __attribute__((packed)) {
	uint8_t clsId;
	uint8_t msgId;
} ubxAckAck;

/* UBX Message */
typedef union {
	ubxAckAck		ackack;
	ubxCfgMsg		cfgmsg;
	ubxCfgNav5		cfgnav5;
	ubxCfgPrt		cfgprt;
	ubxCfgRst		cfgrst;
	ubxCfgRxm		cfgrxm;
	ubxNavPosllh	navposllh;
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

#endif /* UBLOX_H_ */
