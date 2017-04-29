/* All credit goes to 4D systems from original lib https://github.com/4dsystems/ViSi-Genie-Arduino-Library. This is just a port of their code */

#include "GenieDisplayDriver.h"

#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))

void FlushEventQueue(void);

void handleError(void);

void SetLinkState(uint16_t newstate);

uint16_t GetLinkState(void);

bool EnqueueEvent(uint8_t *data);

uint8_t Getchar(void);

uint16_t GetcharSerial(void);

void WaitForIdle(void);

void PushLinkState(uint8_t newstate);

void PopLinkState(void);

void FatalError(void);

void FlushSerialInput(void);

void Resync(void);


//////////////////////////////////////////////////////////////
// A structure to hold up to MAX_GENIE_EVENTS events receive
// from the display
//
EventQueueStruct EventQueue;

//////////////////////////////////////////////////////////////
// Simple 5-deep stack for the link state, this allows
// DoEvents() to save the current state, receive a frame,
// then restore the state
//
uint8_t LinkStates[MAX_LINK_STATES];
//
// Stack pointer
//
uint8_t *LinkState;

//////////////////////////////////////////////////////////////
// Number of mS the GetChar() function will wait before
// giving up on the display
int Timeout;

//////////////////////////////////////////////////////////////
// Number of times we have had a timeout
int Timeouts;

//////////////////////////////////////////////////////////////
// Global error variable
int Error;


uint8_t rxframe_count;

//////////////////////////////////////////////////////////////
// Number of fatal errors encountered
int FatalErrors;

UserEventHandlerPtr UserHandler;
UserBytePtr UserByteReader;
UserDoubleBytePtr UserDoubleByteReader;


static UartWrapper *deviceSerial;

#include <math.h>
#include <string.h>

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

void initGenieDisplayDriver(UartWrapper *wrapper) {

    UserHandler = NULL;
    UserByteReader = NULL;
    UserDoubleByteReader = NULL;
    LinkStates[0] = GENIE_LINK_IDLE;
    LinkState = &LinkStates[0];
    Timeout = TIMEOUT_PERIOD;
    Timeouts = 0;
    Error = ERROR_NONE;
    rxframe_count = 0;
    FatalErrors = 0;
}

////////////////////// GetEventData ////////////////////////
//
// Returns the LSB and MSB of the event's data combined into
// a single uint16
//
// The data is transmitted from the display in big-endian format
// and stored the same so the user can't just access it as an int
// directly from the structure.
//
uint16_t GetEventData(GenieFrame *e) {
    return (e->reportObject.data_msb << 8) + e->reportObject.data_lsb;
}

//////////////////////// GetNextByte ///////////////////////////
//
// Read one byte from the serial device.  Blocking.
//
uint8_t GetNextByte(void) {
    while (deviceSerial->available() < 1) {
        continue;
    }
    return deviceSerial->read();
}

//////////////////////// GetNextDoubleByte ///////////////////////////
//
// Reads two bytes from the serial device and joins them into one
// double byte.  Blocking.
//
uint16_t GetNextDoubleByte(void) {
    uint16_t out;
    while (deviceSerial->available() < 1) {
        continue;
    }
    out = (deviceSerial->read()) << 8;
    out |= deviceSerial->read();
    return out;
}

//////////////////////// Genie::EventIs ///////////////////////////
//
// Compares the cmd, object and index fields of the event's
// structure.
//
// Returns:     TRUE if all the fields match the caller's parms
//              FALSE if any of them don't
//
bool EventIs(GenieFrame *e, uint8_t cmd, uint8_t object, uint8_t index) {
    return (e->reportObject.cmd == cmd &&
            e->reportObject.object == object &&
            e->reportObject.index == index);
}

////////////////////// Genie::WaitForIdle ////////////////////////
//
// Wait for the link to become idle or for the timeout period,
// whichever comes first.
//
void WaitForIdle(void) {
    uint16_t do_event_result;
    long timeout = millis() + Timeout;

    for (; millis() < timeout;) {
        do_event_result = DoEvents(false);

        // if there was a character received from the
        // display restart the timeout because doEvents
        // is in the process of receiving something
        if (do_event_result == GENIE_EVENT_RXCHAR) {
            timeout = millis() + Timeout;
        }

        if (GetLinkState() == GENIE_LINK_IDLE) {
            return;
        }
    }

    Error = ERROR_TIMEOUT;
    handleError();
    return;
}

////////////////////// Genie::PushLinkState //////////////////////
//
// Push a link state onto a FILO stack
//
int linkCount = 0;

void PushLinkState(uint8_t newstate) {
    if (linkCount >= MAX_LINK_STATES) {
        Resync();
    }

    linkCount++;
    LinkState++;
    //if (debugSerial) { *debugSerial << " newstate = " << newstate << " LinkState count = " << linkCount << ", Freemem = " << freeRam() << ", " << (unsigned long)&LinkState[0] << ", rxframe_count = " << rxframe_count << endl; } ;
    SetLinkState(newstate);
}

////////////////////// Genie::PopLinkState //////////////////////
//
// Pop a link state from a FILO stack
//
void PopLinkState(void) {
    //if (debugSerial) { *debugSerial << "PopLinkState\n"; }
    if (LinkState > &LinkStates[0]) {
        *LinkState = 0xFF;
        LinkState--;
        linkCount--;
    }
}

///////////////////////// Genie::DoEvents /////////////////////////
//
// This is the heart of the Genie comms state machine.
//
uint16_t DoEvents(bool DoHandler) {
    uint8_t c;
    static uint8_t rx_data[6];
    static uint8_t checksum = 0;
    c = Getchar();
    static struct MagicReportHeader magicHeader;
    static uint8_t magicByte = 0;

    //if (debugSerial && c != 0xFD) *debugSerial << _HEX(c)<<", "<<"["<<GetLinkState()<<"], ";
    ////////////////////////////////////////////
    //
    // If there are no characters to process and we have
    // queued events call the user's handler function.
    //
    if (Error == ERROR_NOCHAR) {
        if ((EventQueue.n_events > 0) && (UserHandler != NULL) && DoHandler) {
            (UserHandler)();
        }

        return GENIE_EVENT_NONE;
    }

    //if (debugSerial) { *debugSerial << "Freemem = " << freeRam()<< endl; } ;
    //return GENIE_EVENT_RXCHAR; // debug

    ///////////////////////////////////////////
    //
    // Main state machine
    //

    switch (GetLinkState()) {
        case GENIE_LINK_IDLE:
            switch (c) {
                case GENIE_REPORT_EVENT:
                    // event frame out of the blue, set the link state
                    // and fall through to the frame-accumulate code
                    // at the end of this function
                    PushLinkState(GENIE_LINK_RXEVENT);
                    break;

                case GENIEM_REPORT_BYTES:
                    magicByte = 0;
                    PushLinkState(GENIE_LINK_RXMBYTES);
                    break;

                case GENIEM_REPORT_DBYTES:
                    magicByte = 0;
                    PushLinkState(GENIE_LINK_RXMDBYTES);
                    break;

                default:
                    // error, bad character, no other character
                    // is acceptable in this state
                    return GENIE_EVENT_RXCHAR;
            }

            break;

        case GENIE_LINK_WFAN:
            switch (c) {
                case GENIE_ACK:
                    PopLinkState();
                    return GENIE_EVENT_RXCHAR;

                case GENIE_NAK:
                    PopLinkState();
                    Error = ERROR_NAK;
                    handleError();
                    return GENIE_EVENT_RXCHAR;

                case GENIE_REPORT_EVENT:
                    // event frame out of the blue while waiting for an ACK
                    // save/set the link state and fall through to the
                    // frame-accumulate code at the end of this function
                    PushLinkState(GENIE_LINK_RXEVENT);
                    break;

                case GENIEM_REPORT_BYTES:
                    magicByte = 0;
                    PushLinkState(GENIE_LINK_RXMBYTES);
                    break;

                case GENIEM_REPORT_DBYTES:
                    magicByte = 0;
                    PushLinkState(GENIE_LINK_RXMDBYTES);
                    break;

                case GENIE_REPORT_OBJ:
                default:
                    // error, bad character
                    return GENIE_EVENT_RXCHAR;
            }

            break;

        case GENIE_LINK_WF_RXREPORT: // waiting for the first byte of a report
            switch (c) {
                case GENIE_REPORT_EVENT:
                    // event frame out of the blue while waiting for the first
                    // byte of a report frame
                    // save/set the link state and fall through to the
                    // frame-accumulate code at the end of this function
                    PushLinkState(GENIE_LINK_RXEVENT);
                    break;

                case GENIEM_REPORT_BYTES:
                    magicByte = 0;
                    PushLinkState(GENIE_LINK_RXMBYTES);
                    break;

                case GENIEM_REPORT_DBYTES:
                    magicByte = 0;
                    PushLinkState(GENIE_LINK_RXMDBYTES);
                    break;

                case GENIE_REPORT_OBJ:
                    // first byte of a report frame
                    // replace the GENIE_LINK_WF_RXREPORT link state
                    // with GENIE_LINK_RXREPORT to indicate that we
                    // are now receiving a report frame
                    PopLinkState();
                    PushLinkState(GENIE_LINK_RXREPORT);
                    break;

                case GENIE_ACK:
                case GENIE_NAK:
                default:
                    // error, bad character
                    return GENIE_EVENT_RXCHAR;
                    //              break;
            }

        case GENIE_LINK_RXREPORT:       // already receiving report
        case GENIE_LINK_RXEVENT:        // already receiving event
        case GENIE_LINK_RXMBYTES:       // already receiving magic bytes
        case GENIE_LINK_RXMDBYTES:      // already receiving mahic doublebytes
        default:
            break;
    }

    ///////////////////////////////////////////////////////
    // We get here if we are in the process of receiving
    // a report or event frame. Accumulate GENIE_FRAME_SIZE
    // bytes into a local buffer then queue them as a frame
    // into the event queue
    //
    if (GetLinkState() == GENIE_LINK_RXREPORT ||
        GetLinkState() == GENIE_LINK_RXEVENT) {
        checksum = (rxframe_count == 0) ? c : checksum ^ c;
        rx_data[rxframe_count] = c;

        if (rxframe_count == GENIE_FRAME_SIZE - 1) {
            // all bytes received, if the CS is good
            // queue the frame and restore the link state
            if (checksum == 0) {
                EnqueueEvent(rx_data);
                rxframe_count = 0;
                // revert the link state to whatever it was before
                // we started accumulating this frame
                PopLinkState();
                return GENIE_EVENT_RXCHAR;
            } else {
                Error = ERROR_BAD_CS;
                handleError();
            }
        }

        rxframe_count++;
        return GENIE_EVENT_RXCHAR;
    }

    ///////////////////////////////////////////////////////
    // We get here if we are in the process of receiving
    // a magic report.  When the header has been received
    // trigger the byte or double-byte handler to receive
    // the rest of the data.
    //
    if (GetLinkState() == GENIE_LINK_RXMBYTES ||
        GetLinkState() == GENIE_LINK_RXMDBYTES) {

        switch (magicByte) {
            case 0:
                magicHeader.cmd = c;
                magicByte++;
                break;
            case 1:
                magicHeader.index = c;
                magicByte++;
                break;
            case 2:
                magicHeader.length = c;
                magicByte++;
                if (magicHeader.cmd == GENIEM_REPORT_BYTES) {
                    if (UserByteReader != NULL) {
                        UserByteReader(magicHeader.index, magicHeader.length);
                    } else {
                        // No handler defined - we need to sink the bytes.
                        while (--magicHeader.length > 0) {
                            (void) GetNextByte();
                        }
                    }
                } else if (magicHeader.cmd == GENIEM_REPORT_DBYTES) {
                    if (UserDoubleByteReader != NULL) {
                        UserDoubleByteReader(magicHeader.index, magicHeader.length);
                    } else {
                        // No handler defined - we need to sink the bytes.
                        while (--magicHeader.length > 0) {
                            (void) GetNextDoubleByte();
                        }
                    }
                }
                // Now we want to discard the checksum. We don't yet
                // know what has been going on with the data, so we
                // can't calculate the checksum.
                (void) GetNextByte();
                PopLinkState();
                break;
        }
        return GENIE_EVENT_RXCHAR;
    }
    return GENIE_EVENT_RXCHAR; // What should we really return here?!
}

//////////////////////// Genie::Getchar //////////////////////////
//
// Get a character from the selected Genie serial port
//
// Returns: ERROR_NOHANDLER if an Rx handler has not
//              been defined
//          ERROR_NOCHAR if no bytes have beeb received
//          The char if there was one to get
// Sets:    Error with any errors encountered
//
uint8_t Getchar(void) {
    Error = ERROR_NONE;
    return GetcharSerial();
}

///////////////////////////////////////////////////////////////////
// Serial port 0 (Serial) Rx  handler
// Return ERROR_NOCHAR if no character or the char in the lower
// byte if there is.
//
uint16_t GetcharSerial(void) {
#ifdef SERIAL

    if (deviceSerial->available() == 0) {
        Error = ERROR_NOCHAR;
        return ERROR_NOCHAR;
    }

    return (uint16_t) deviceSerial->read() & 0xFF;
#endif
    return 0;
}


/////////////////// Genie::FatalError ///////////////////////
//
void FatalError(void) {
    if (FatalErrors++ > MAX_GENIE_FATALS) {
        //      *LinkState = GENIE_LINK_SHDN;
        //      Error = ERROR_NODISPLAY;
    }
}

///////////////// Genie::FlushSerialInput ///////////////////
//
// Removes and discards all characters from the currently
// used serial port's Rx buffer.
//
void FlushSerialInput(void) {
    while (deviceSerial->read() >= 0);
}

/////////////////////// Resync //////////////////////////
//
// This function does nothing for RESYNC_PERIOD to allow the display
// time to stop talking, then it flushes everything so the link
// can start again.
//
// Untested, will need work I'm sure.
//
void Resync(void) {
    //for (long timeout = millis() + RESYNC_PERIOD ; millis() < timeout;) {};
    FlushSerialInput();
    FlushEventQueue();
    Timeouts = 0;
    linkCount = 0;
    LinkState = &LinkStates[0];
    *LinkState = GENIE_LINK_IDLE;
}

///////////////////////// handleError /////////////////////////
//
// So far really just a debugging aid, but can be enhanced to
// help recover from errors.
//
void handleError(void) {
    //if (debugSerial) { *debugSerial << "Handle Error Called!\n"; }
}

////////////////////// Genie::FlushEventQueue ////////////////////
//
// Reset all the event queue variables and start from scratch.
//
void FlushEventQueue(void) {
    EventQueue.rd_index = 0;
    EventQueue.wr_index = 0;
    EventQueue.n_events = 0;
}

////////////////////// DequeueEvent ///////////////////
//
// Copy the bytes from a queued input event to a buffer supplied
// by the caller.
//
// Parms:   genieFrame * buff, a pointer to the user's buffer
//
// Returns: TRUE if there was an event to copy
//          FALSE if not
//
bool DequeueEvent(GenieFrame *buff) {
    if (EventQueue.n_events > 0) {
        memcpy(buff, &EventQueue.frames[EventQueue.rd_index],
               GENIE_FRAME_SIZE);
        EventQueue.rd_index++;
        EventQueue.rd_index &= MAX_GENIE_EVENTS - 1;
        EventQueue.n_events--;
        return TRUE;
    }

    return FALSE;
}

////////////////////// Genie::EnqueueEvent ///////////////////
//
// Copy the bytes from a buffer supplied by the caller
// to the input queue
//
// Parms:   uint8_t * data, a pointer to the user's data
//
// Returns: TRUE if there was an empty location in the queue
//              to copy the data into
//          FALSE if not
// Sets:    ERROR_REPLY_OVR if there was no room in the queue
//
bool EnqueueEvent(uint8_t *data) {
    if (EventQueue.n_events < MAX_GENIE_EVENTS - 2) {
        int i, j;
        bool fnd = false;
        j = EventQueue.wr_index;
        for (i = EventQueue.n_events; i > 0; i--) {
            j--;
            if (j < 0)
                j = MAX_GENIE_EVENTS - 1;
            if ((EventQueue.frames[j].reportObject.cmd == data[0])
                && (EventQueue.frames[j].reportObject.object == data[1])
                && (EventQueue.frames[j].reportObject.index == data[2])) {
                EventQueue.frames[j].reportObject.data_msb = data[3];
                EventQueue.frames[j].reportObject.data_lsb = data[4];
                fnd = true;
                break;
            }
        }
        if (!fnd) {
            memcpy(&EventQueue.frames[EventQueue.wr_index], data,
                   GENIE_FRAME_SIZE);
            EventQueue.wr_index++;
            EventQueue.wr_index &= MAX_GENIE_EVENTS - 1;
            EventQueue.n_events++;
            //if (debugSerial) { *debugSerial << "Enque Event " << _HEX(*data) << ", count = " << EventQueue.n_events << endl; }
            return TRUE;
        }
    } else {
        Error = ERROR_REPLY_OVR;
        handleError();
        return FALSE;
    }
    return false;
}

//////////////////////// Genie::ReadObject ///////////////////////
//
// Send a read object command to the Genie display. Note that this
// function does not wait for the reply, that will be read in due
// course by DoEvents() and subsequently by the user's event
// handler.
//
bool ReadObject(uint16_t object, uint16_t index) {
    uint8_t checksum;
    // Discard any pending reply frames
    //FlushEventQueue();    // Removed due to preventing more than 2 readObjects being queued
    WaitForIdle();
    Error = ERROR_NONE;
    deviceSerial->write((uint8_t) GENIE_READ_OBJ);
    checksum = GENIE_READ_OBJ;
    deviceSerial->write(object);
    checksum ^= object;
    deviceSerial->write(index);
    checksum ^= index;
    deviceSerial->write(checksum);
    PushLinkState(GENIE_LINK_WF_RXREPORT);
    return TRUE;
}

///////////////////// Genie::SetLinkState ////////////////////////
//
// Set the logical state of the link to the display.
//
// Parms:   uint16_t newstate, a value to be written to the
//              link's Genie::LinkState variable. Valid values are
//      GENIE_LINK_IDLE         0
//      GENIE_LINK_WFAN         1 // waiting for Ack or Nak
//      GENIE_LINK_WF_RXREPORT  2 // waiting for a report frame
//      GENIE_LINK_RXREPORT     3 // receiving a report frame
//      GENIE_LINK_RXEVENT      4 // receiving an event frame
//      GENIE_LINK_SHDN         5
//
void SetLinkState(uint16_t newstate) {
    *LinkState = newstate;

    if (newstate == GENIE_LINK_RXREPORT || \
            newstate == GENIE_LINK_RXEVENT) {
        rxframe_count = 0;
    }
}

uint16_t GetLinkState(void) {
    return *LinkState;
}

uint16_t WriteObject(uint16_t object, uint16_t index, uint16_t data) {
    uint16_t msb, lsb;
    uint8_t checksum;
    WaitForIdle();
    lsb = lowByte(data);
    msb = highByte(data);
    Error = ERROR_NONE;
    deviceSerial->write(GENIE_WRITE_OBJ);
    checksum = GENIE_WRITE_OBJ;
    deviceSerial->write(object);
    checksum ^= object;
    deviceSerial->write(index);
    checksum ^= index;
    deviceSerial->write(msb);
    checksum ^= msb;
    deviceSerial->write(lsb);
    checksum ^= lsb;
    deviceSerial->write(checksum);
    PushLinkState(GENIE_LINK_WFAN);
    return 0;
}

/* 0 or 1, or 0-15 on uLCD */
void WriteContrast(uint16_t value) {
    unsigned int checksum;
    WaitForIdle();
    deviceSerial->write(GENIE_WRITE_CONTRAST);
    checksum = GENIE_WRITE_CONTRAST;
    deviceSerial->write(value);
    checksum ^= value;
    deviceSerial->write(checksum);
    PushLinkState(GENIE_LINK_WFAN);
}

uint16_t WriteStr(uint16_t index, char *string) {
    char *p;
    unsigned int checksum;
    int len = strlen(string);

    if (len > 255) {
        return -1;
    }
    WaitForIdle();
    deviceSerial->write(GENIE_WRITE_STR);
    checksum = GENIE_WRITE_STR;
    deviceSerial->write(index);
    checksum ^= index;
    deviceSerial->write((unsigned char) len);
    checksum ^= len;
    for (p = string; *p; ++p) {
        deviceSerial->write(*p);
        checksum ^= *p;
    }
    deviceSerial->write(checksum);
    PushLinkState(GENIE_LINK_WFAN);
    return 0;
}

uint16_t WriteStrU(uint16_t index, uint16_t *string) {
    uint16_t *p;
    unsigned int checksum;
    int len = 0;
    p = string;
    while (*p++) {
        len++;
    }
    if (len > 255) {
        return -1;
    }
    WaitForIdle();
    deviceSerial->write(GENIE_WRITE_STRU);
    checksum = GENIE_WRITE_STRU;
    deviceSerial->write(index);
    checksum ^= index;
    deviceSerial->write((unsigned char) (len));
    checksum ^= (len);
    p = string;
    while (*p) {
        deviceSerial->write(*p >> 8);
        checksum ^= *p >> 8;
        deviceSerial->write(*p);
        checksum ^= *p++ & 0xff;
    }
    deviceSerial->write(checksum);
    PushLinkState(GENIE_LINK_WFAN);
    return 0;
}

void AttachEventHandler(UserEventHandlerPtr handler) {
    UserHandler = handler;
}

void AttachMagicByteReader(UserBytePtr handler) {
    UserByteReader = handler;
}

void AttachMagicDoubleByteReader(UserDoubleBytePtr handler) {
    UserDoubleByteReader = handler;
}

/*
TODO: PORT THIS!
void Genie::Begin (Stream &serial) {
    deviceSerial = &serial;
    PushLinkState(GENIE_LINK_IDLE);
    FlushEventQueue();
}
*/
uint16_t WriteMagicBytes(uint16_t index, uint8_t *bytes, uint16_t len) {
    unsigned int checksum;

    if (len > 255) {
        return -1;
    }

    WaitForIdle();
    deviceSerial->write(GENIEM_WRITE_BYTES);
    checksum = GENIEM_WRITE_BYTES;
    deviceSerial->write(index);
    checksum ^= index;
    deviceSerial->write((unsigned char) len);
    checksum ^= len;

    for (int i = 0; i < len; i++) {
        deviceSerial->write(bytes[i]);
        checksum ^= bytes[i];
    }

    deviceSerial->write(checksum);
    PushLinkState(GENIE_LINK_WFAN);
    return 0;
}

uint16_t WriteMagicDBytes(uint16_t index, uint16_t *shorts, uint16_t len) {
    unsigned int checksum;

    if (len > 255) {
        return -1;
    }
    WaitForIdle();
    deviceSerial->write(GENIEM_WRITE_DBYTES);
    checksum = GENIEM_WRITE_DBYTES;
    deviceSerial->write(index);
    checksum ^= index;
    deviceSerial->write((unsigned char) (len));
    checksum ^= (len);

    for (int i = 0; i < len; i++) {
        deviceSerial->write(shorts[i] >> 8);
        checksum ^= shorts[i] >> 8;
        deviceSerial->write(shorts[i] & 0xFF);
        checksum ^= shorts[i] & 0xff;
    }

    deviceSerial->write(checksum);
    PushLinkState(GENIE_LINK_WFAN);
    return 0;
}
