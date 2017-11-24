/*
 * ublox.c
 *
 *  Created on: Oct 28, 2017
 *      Author: Milosz Iskrzynski SQ6NTI
 */

#include "ublox.h"

ubxGPSData currentGPSData;
ubxPacket recentCfgResponse;
int recentCfgResponseValid;
UART_HandleTypeDef *gpsUart;

void ubxInit(UART_HandleTypeDef *uart) {
	gpsUart = uart;

	/* Try to reset GPS at modified baud rate first (38400) */
	ubxMessage cfgRstMsg = {
		.cfgrst = {
			.navBbrMask = 0xffff,
			.resetMode = 1,
			.reserved1 = 0
		}
	};
	ubxSendPacket(UBX_CFG, UBX_CFG_RST, sizeof(ubxCfgRst), cfgRstMsg);
	HAL_Delay(800); /* Wait for reset (how long does the reset take?) */

	/* Switch to default (9600) and try to reset GPS again in case previous try failed */
	HAL_UART_DMAStop(gpsUart);
	UART_ReInit(gpsUart, 9600);
	ubxSendPacket(UBX_CFG, UBX_CFG_RST, sizeof(ubxCfgRst), cfgRstMsg);
	HAL_Delay(800); /* wait for reset (how long does the reset take?) */

	/* Configure port for 38400 baud rate and switch to the new settings */
	ubxMessage cfgPrtMsg = {
		.cfgprt = {
			.portID = 1,
			.reserved1 = 0,
			.txReady = 0,
			.mode = 0b00100011000000,
			.baudRate = 38400,
			.inProtoMask = 1,
			.outProtoMask = 1,
			.reserved2 = {0,0}
		}
	};
	ubxSendPacket(UBX_CFG, UBX_CFG_PRT, sizeof(ubxCfgPrt), cfgPrtMsg);
	UART_ReInit(gpsUart, 38400);
	HAL_Delay(100); /* wait until ublox re-config is done (what should be the exact time?) */
	//ubxSendPacket(UBX_CFG, UBX_CFG_PRT, sizeof(ubxCfgPrt), cfgPrtMsg);

	/* Configure NAV-POSLLH outgoing messages */
	ubxMessage cfgMsgMsg = {
		.cfgmsg = {
			.msgClass = 0x01,
			.msgID = 0x02,
			.rate = 5
		}
	};
	do {
		ubxSendPacket(UBX_CFG, UBX_CFG_MSG, sizeof(ubxCfgMsg), cfgMsgMsg);
	} while (!ubxResponseWait(20));

	/* Configure NAV-STATUS outgoing messages */
	cfgMsgMsg.cfgmsg.msgID = 0x03;
	do {
		ubxSendPacket(UBX_CFG, UBX_CFG_MSG, sizeof(ubxCfgMsg), cfgMsgMsg);
	} while (!ubxResponseWait(20));
}

void ubxRxByte(uint8_t data) {
	static uint8_t dataPos = 0;
	static uint8_t ubxSync = 0;
	static uint8_t ubxHeaderReady = 0;
	static uint16_t msgPos = 0;
	static uint16_t lengthWithHeader = 0;
	static ubxPacket packet;
	static const ubxPacket EmptyPacket;

	if (!ubxSync) {
		if (dataPos == 0 && data == UBX_SYNC1) { /* Check fist sync byte of UBX Packet */
			packet.header.syncChar1 = UBX_SYNC1;
			dataPos = 1;
		} else if (dataPos == 1 && data == UBX_SYNC2) { /* Check second sync byte of UBX Packet */
			packet.header.syncChar2 = UBX_SYNC2;
			ubxSync = 1;
			dataPos = 2;
		} else { /* Reset state if sync bytes not received in the above sequence */
			dataPos = 0;
		}
	} else { /* Synchronization is now assumed */
		if (!ubxHeaderReady) {
			if (dataPos == 2) {
				packet.header.messageClass = data;
			} else if (dataPos == 3) {
				packet.header.messageId = data;
			} else if (dataPos == 4) {
				packet.header.payloadLength = data;
			} else if (dataPos == 5) {
				packet.header.payloadLength += data << 8; /* multiple of 256) */
				lengthWithHeader = sizeof(ubxHeader) + packet.header.payloadLength;
				ubxHeaderReady = 1;
			}
		} else { /* Header is now ready, start reading payload and checksum */
			if (dataPos < lengthWithHeader) {
				((uint8_t *)&packet.message)[msgPos] = data;
				msgPos++;
			} else if (dataPos == lengthWithHeader) { /* read ck_a */
				packet.checksum.ck_a = data;
			} else if (dataPos == lengthWithHeader+1) { /* read ck_b */
				packet.checksum.ck_b = data;
			} else { /* Packet is now ready for further processing */
				/* Static status cleanup */
				dataPos = 0;
				ubxSync = 0;
				ubxHeaderReady = 0;
				msgPos = 0;
				lengthWithHeader = 0;

				ubxProcessPacket(&packet);
				packet = EmptyPacket;
			}
		}
		dataPos++;
	}
}

void ubxProcessPacket(const ubxPacket *packet) {
	/* Checksum validation */
	ubxChecksum checksum = ubxCalcChecksum(packet);
	if (checksum.ck_a != packet->checksum.ck_a || checksum.ck_b != packet->checksum.ck_b) {
		return;
	}

	/* Select action based on message class and ID */
	if (packet->header.messageClass == UBX_ACK) {
		recentCfgResponse = *packet;
		recentCfgResponseValid = 1;
	} else if (packet->header.messageClass == UBX_NAV) {
		if (packet->header.messageId == UBX_NAV_POSLLH) {
			currentGPSData.lat = packet->message.navposllh.lat;
			currentGPSData.lon = packet->message.navposllh.lon;
			currentGPSData.alt = packet->message.navposllh.hMSL;
		} else if (packet->header.messageId == UBX_NAV_TIMEUTC) {
			currentGPSData.year = packet->message.navtimeutc.year;
			currentGPSData.month = packet->message.navtimeutc.month;
			currentGPSData.day = packet->message.navtimeutc.day;
			currentGPSData.hour = packet->message.navtimeutc.hour;
			currentGPSData.minute = packet->message.navtimeutc.min;
			currentGPSData.second = packet->message.navtimeutc.sec;
		} else if (packet->header.messageId == UBX_NAV_STATUS) {
			currentGPSData.fix = packet->message.navstatus.flags & 1;
		}
	}
}

int ubxResponseWait(int timeout) {
	while (!recentCfgResponseValid) {
		HAL_Delay(10);
		if (timeout > 0) {
			timeout--;
		} else {
			return 0;
		}
	}

	if (recentCfgResponse.header.messageId == UBX_ACK_ACK) {
		return 1;
	}

	return 0;
}

void ubxSendPacket(uint8_t messageClass, uint8_t messageId, uint16_t payloadLength, ubxMessage message) {
	ubxPacket packet = {
		.header = {
			.syncChar1 = UBX_SYNC1,
			.syncChar2 = UBX_SYNC2,
			.messageClass = messageClass,
			.messageId = messageId,
			.payloadLength = payloadLength
		},
		.message = message,
		.checksum = {0,0}
	};
	packet.checksum = ubxCalcChecksum(&packet);

	if (packet.header.messageClass == UBX_CFG) {
		/* When sending CFG message, we expect ACK or NAK in return. Invalidate previous CFG esponse. */
		recentCfgResponseValid = 0;
	}

	ubxTxPacket(packet);
}

void ubxTxPacket(ubxPacket packet) {
	UART_TxStart(gpsUart);
	while (HAL_UART_Transmit_DMA(gpsUart, (void*)&packet.header, sizeof(ubxHeader)) == HAL_BUSY);
	while (!UART_TxFinished(gpsUart));
	UART_TxStart(gpsUart);
	while (HAL_UART_Transmit_DMA(gpsUart, (void*)&packet.message, packet.header.payloadLength) == HAL_BUSY);
	while (!UART_TxFinished(gpsUart));
	UART_TxStart(gpsUart);
	while (HAL_UART_Transmit_DMA(gpsUart, (void*)&packet.checksum, sizeof(ubxChecksum)) == HAL_BUSY);
	while (!UART_TxFinished(gpsUart));
}

ubxChecksum ubxCalcChecksum(const ubxPacket *packet) {
	uint8_t *message = (uint8_t *)&packet->message;
	ubxChecksum checksum = {0, 0};
	uint8_t i;

	checksum.ck_a += packet->header.messageClass;
	checksum.ck_b += checksum.ck_a;
	checksum.ck_a += packet->header.messageId;
	checksum.ck_b += checksum.ck_a;
	checksum.ck_a += packet->header.payloadLength & 0xff;
	checksum.ck_b += checksum.ck_a;
	checksum.ck_a += packet->header.payloadLength >> 8;
	checksum.ck_b += checksum.ck_a;


	for (i=0; i < packet->header.payloadLength; i++){
		checksum.ck_a += message[i];
		checksum.ck_b += checksum.ck_a;
	}

	return checksum;
}

ubxGPSData ubxLastGPSData() {
	uint32_t prim;
	ubxGPSData gpsData;

	/* Get PRIMASK (current interrupt status) and disable global interrupts to preserve data integrity */
	prim = __get_PRIMASK();
	__disable_irq();

	gpsData = currentGPSData;

	/* Re-enable global interrupts, but only if PRIMASK register not set */
	if (!prim) {
		__enable_irq();
	}

	return gpsData;
}
