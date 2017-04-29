/* Port from https://github.com/4dsystems/ViSi-Genie-Arduino-Library. All credit to 4D systems, this is simply a port */

#ifndef GENIE_DISPLAY_DRIVER_H
#define GENIE_DISPLAY_DRIVER_H

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct UartWrapper {
    bool (*available)(void);
    uint8_t (*read)(void);
    void (*write)(uint32_t val);
    void (*millis)(uint32_t millis);
} UartWrapper;

#undef GENIE_DEBUG
#define GENIE_VERSION    "06-10-2015"
#define GENIE_ACK               0x06
#define GENIE_NAK               0x15
#define TIMEOUT_PERIOD          1000
#define RESYNC_PERIOD           100
#define GENIE_READ_OBJ          0
#define GENIE_WRITE_OBJ         1
#define GENIE_WRITE_STR         2
#define GENIE_WRITE_STRU        3
#define GENIE_WRITE_CONTRAST    4
#define GENIE_REPORT_OBJ        5
#define GENIE_REPORT_EVENT      7
#define GENIEM_WRITE_BYTES      8
#define GENIEM_WRITE_DBYTES     9
#define GENIEM_REPORT_BYTES     10
#define GENIEM_REPORT_DBYTES    11
#define GENIE_OBJ_DIPSW         0
#define GENIE_OBJ_KNOB          1
#define GENIE_OBJ_ROCKERSW      2
#define GENIE_OBJ_ROTARYSW      3
#define GENIE_OBJ_SLIDER        4
#define GENIE_OBJ_TRACKBAR      5
#define GENIE_OBJ_WINBUTTON     6
#define GENIE_OBJ_ANGULAR_METER 7
#define GENIE_OBJ_COOL_GAUGE    8
#define GENIE_OBJ_CUSTOM_DIGITS 9
#define GENIE_OBJ_FORM          10
#define GENIE_OBJ_GAUGE         11
#define GENIE_OBJ_IMAGE         12
#define GENIE_OBJ_KEYBOARD      13
#define GENIE_OBJ_LED           14
#define GENIE_OBJ_LED_DIGITS    15
#define GENIE_OBJ_METER         16
#define GENIE_OBJ_STRINGS       17
#define GENIE_OBJ_THERMOMETER   18
#define GENIE_OBJ_USER_LED      19
#define GENIE_OBJ_VIDEO         20
#define GENIE_OBJ_STATIC_TEXT   21
#define GENIE_OBJ_SOUND         22
#define GENIE_OBJ_TIMER         23
#define GENIE_OBJ_SPECTRUM      24
#define GENIE_OBJ_SCOPE         25
#define GENIE_OBJ_TANK          26
#define GENIE_OBJ_USERIMAGES    27
#define GENIE_OBJ_PINOUTPUT     28
#define GENIE_OBJ_PININPUT      29
#define GENIE_OBJ_4DBUTTON      30
#define GENIE_OBJ_ANIBUTTON     31
#define GENIE_OBJ_COLORPICKER   32
#define GENIE_OBJ_USERBUTTON    33

#define GENIE_FRAME_SIZE        6

typedef struct FrameReportObj {
    uint8_t cmd;
    uint8_t object;
    uint8_t index;
    uint8_t data_msb;
    uint8_t data_lsb;
} FrameReportObj;

typedef struct MagicReportHeader {
    uint8_t cmd;
    uint8_t index;
    uint8_t length;
} MagicReportHeader;

typedef union GenieFrame {
    uint8_t bytes[GENIE_FRAME_SIZE];
    FrameReportObj reportObject;
} GenieFrame;

#define MAX_GENIE_EVENTS    16    // MUST be a power of 2
#define MAX_GENIE_FATALS    10
#define MAX_LINK_STATES     20

typedef struct EventQueueStruct {
    GenieFrame frames[MAX_GENIE_EVENTS];
    uint8_t rd_index;
    uint8_t wr_index;
    uint8_t n_events;
} EventQueueStruct;

typedef void        (*UserEventHandlerPtr)(void);
typedef void        (*UserBytePtr)(uint8_t, uint8_t);
typedef void        (*UserDoubleBytePtr)(uint8_t, uint8_t);

void initGenieDisplayDriver(UartWrapper *wrapper);

bool ReadObject(uint16_t object, uint16_t index);

uint16_t WriteObject(uint16_t object, uint16_t index, uint16_t data);

void WriteContrast(uint16_t value);

uint16_t WriteStr(uint16_t index, char *string);

uint16_t WriteStrU(uint16_t index, uint16_t *string);

bool EventIs(GenieFrame *e, uint8_t cmd, uint8_t object, uint8_t index);

uint16_t GetEventData(GenieFrame *e);

bool DequeueEvent(GenieFrame *buff);

uint16_t DoEvents(bool DoHandler);

void AttachEventHandler(UserEventHandlerPtr userHandler);

void AttachMagicByteReader(UserBytePtr userHandler);

void AttachMagicDoubleByteReader(UserDoubleBytePtr userHandler);

void pulse(int pin);

/* Genie Magic functions (ViSi-Genie Pro Only) */
uint16_t WriteMagicBytes(uint16_t index, uint8_t *bytes, uint16_t len);

uint16_t WriteMagicDBytes(uint16_t index, uint16_t *bytes, uint16_t len);

uint8_t GetNextByte(void);

uint16_t GetNextDoubleByte(void);

#ifndef TRUE
#define TRUE    (1==1)
#define FALSE    (!TRUE)
#endif

#define ERROR_NONE           0
#define ERROR_TIMEOUT       -1    // 255  0xFF
#define ERROR_NOHANDLER     -2    // 254  0xFE
#define ERROR_NOCHAR        -3    // 253  0xFD
#define ERROR_NAK           -4    // 252  0xFC
#define ERROR_REPLY_OVR     -5    // 251  0xFB
#define ERROR_RESYNC        -6    // 250  0xFA
#define ERROR_NODISPLAY     -7    // 249  0xF9
#define ERROR_BAD_CS        -8    // 248  0xF8

#define GENIE_LINK_IDLE           0
#define GENIE_LINK_WFAN           1 // waiting for Ack or Nak
#define GENIE_LINK_WF_RXREPORT    2 // waiting for a report frame
#define GENIE_LINK_RXREPORT       3 // receiving a report frame
#define GENIE_LINK_RXEVENT        4 // receiving an event frame
#define GENIE_LINK_SHDN           5
#define GENIE_LINK_RXMBYTES       6 // receiving magic bytes
#define GENIE_LINK_RXMDBYTES      7 // receiving magic dbytes

#define GENIE_EVENT_NONE    0
#define GENIE_EVENT_RXCHAR  1

#endif
