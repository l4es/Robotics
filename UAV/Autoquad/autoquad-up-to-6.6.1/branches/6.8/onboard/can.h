/*
    This file is part of AutoQuad.

    AutoQuad is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    AutoQuad is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with AutoQuad.  If not, see <http://www.gnu.org/licenses/>.

    Copyright © 2011, 2012, 2013  Bill Nesbitt
*/

#ifndef _can_h
#define _can_h

// Logical Communications Channel
// 2 bits [28:27]
#define CAN_LCC_MASK	    ((uint32_t)0x3<<30)
#define CAN_LCC_EXCEPTION   ((uint32_t)0x0<<30)
#define CAN_LCC_HIGH	    ((uint32_t)0x1<<30)
#define CAN_LCC_NORMAL	    ((uint32_t)0x2<<30)
#define CAN_LCC_INFO	    ((uint32_t)0x3<<30)

// Target Type
// 1 bit [26:26]
#define CAN_TT_MASK	    ((uint32_t)0x1<<29)
#define CAN_TT_GROUP	    ((uint32_t)0x0<<29)
#define CAN_TT_NODE	    ((uint32_t)0x1<<29)

// Function ID
// 4 bits [25:22]
#define CAN_FID_MASK	    ((uint32_t)0xf<<25)
#define CAN_FID_RESET_BUS   ((uint32_t)0x0<<25)
#define CAN_FID_ACK	    ((uint32_t)0x1<<25)
#define CAN_FID_NACK	    ((uint32_t)0x2<<25)
#define CAN_FID_CMD	    ((uint32_t)0x3<<25)
#define CAN_FID_GET	    ((uint32_t)0x4<<25)
#define CAN_FID_SET	    ((uint32_t)0x5<<25)
#define CAN_FID_REPLY	    ((uint32_t)0x6<<25)
#define CAN_FID_REQ_ADDR    ((uint32_t)0x7<<25)
#define CAN_FID_GRANT_ADDR  ((uint32_t)0x8<<25)
#define CAN_FID_ERROR	    ((uint32_t)0x9<<25)
#define CAN_FID_PING	    ((uint32_t)0xa<<25)

// Data Object Code
// 6 bits [21:16]
#define CAN_DOC_MASK	    ((uint32_t)0x3f<<19)

// Source ID
// 5 bits [15:11]
#define CAN_SID_MASK	    ((uint32_t)0x1f<<14)

// Target ID
// 5 bits [10:6]
#define CAN_TID_MASK	    ((uint32_t)0x1f<<9)

// Sequence ID
// 6 bits [5:0]
#define CAN_SEQ_MASK	    ((uint32_t)0x3f<<3)

#define CAN_TIMEOUT	    1000			    // ms

enum {
    CAN_TYPE_ESC = 1,
    CAN_TYPE_SERVO,
    CAN_TYPE_SENSOR,
    CAN_TYPE_LED,
    CAN_TYPE_OSD,
    CAN_TYPE_UART,
    CAN_TYPE_HUB
};

enum {
    CAN_CMD_DISARM = 1,
    CAN_CMD_ARM,
    CAN_CMD_START,
    CAN_CMD_STOP,
    CAN_CMD_SETPOINT10,
    CAN_CMD_SETPOINT12,
    CAN_CMD_SETPOINT16,
    CAN_CMD_RPM,
    CAN_CMD_CFG_READ,
    CAN_CMD_CFG_WRITE,
    CAN_CMD_CFG_DEFAULT,
    CAN_CMD_TELEM_RATE,
    CAN_CMD_TELEM_VALUE,
    CAN_CMD_BEEP,
    CAN_CMD_POS,
    CAN_CMD_USER_DEFINED,
    CAN_CMD_RESET
};

enum {
    CAN_DATA_GROUP = 1,
    CAN_DATA_TYPE,
    CAN_DATA_ID,
    CAN_DATA_INPUT_MODE,
    CAN_DATA_RUN_MODE,
    CAN_DATA_STATE,
    CAN_DATA_PARAM,
    CAN_DATA_TELEM,
    CAN_DATA_VERSION
};

typedef struct {
    uint32_t id;
    uint32_t *data;
    uint8_t sid;
    uint8_t tid;
    uint8_t seq;
    uint8_t doc;
} canPacket_t;

typedef struct {
    unsigned int value1 : 10;
    unsigned int value2 : 10;
    unsigned int value3 : 10;
    unsigned int value4 : 10;
    unsigned int value5 : 10;
    unsigned int value6 : 10;
    unsigned int unused : 4;
} __attribute__((packed)) canGroup10_t;

typedef struct {
    unsigned int value1 : 12;
    unsigned int value2 : 12;
    unsigned int value3 : 12;
    unsigned int value4 : 12;
    unsigned int value5 : 12;
    unsigned int unused : 4;
} __attribute__((packed)) canGroup12_t;

typedef struct {
    uint16_t value1;
    uint16_t value2;
    uint16_t value3;
    uint16_t value4;
} __attribute__((packed)) canGroup16_t;

typedef struct {
    uint32_t uuid;
    uint8_t nodeId;
    uint8_t type;
    uint8_t canId;
    uint8_t groupId;
    uint8_t subgroupId;
} canNodes_t;

typedef struct {
    uint32_t mailboxFull;
    canNodes_t nodes[(CAN_TID_MASK>>9)+1];
    volatile uint8_t responses[64];
    uint8_t responseData[64*8];
    uint8_t nextNode;
    uint8_t seqId;
    uint8_t initialized;
} canStruct_t;

extern canStruct_t canData;

extern void canInit(void);
extern void canLowLevelInit(void);
extern void canCheckMessage(void);
extern char *canGetVersion(uint8_t tid);
extern float *canGetParam(uint8_t tid, uint16_t paramId);
extern uint8_t *canSetParam(uint32_t tt, uint8_t tid, uint16_t paramId, float value);
extern uint8_t *canGetState(uint8_t tid);
extern uint8_t *canSetGroup(uint8_t tid, uint8_t gid, uint8_t sgid);
extern void canCommandPos(uint8_t tid, float angle);
extern uint8_t *canCommandBeep(uint32_t tt, uint8_t tid, uint16_t freq, uint16_t dur);
extern uint8_t *canSetRunMode(uint32_t tt, uint8_t tid, uint8_t mode);
extern void canCommandArm(uint32_t tt, uint8_t tid);
extern void canCommandDisarm(uint32_t tt, uint8_t tid);
extern uint8_t *canCommandConfigWrite(uint32_t tt, uint8_t tid);
extern uint8_t *canCommandStart(uint32_t tt, uint8_t tid);
extern uint8_t *canCommandStop(uint32_t tt, uint8_t tid);
extern canNodes_t *canFindNode(uint8_t type, uint8_t canId);
extern void canCommandSetpoint16(uint8_t tid, uint8_t *data);

#endif