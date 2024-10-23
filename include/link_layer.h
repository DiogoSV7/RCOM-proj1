// Link layer header.
// NOTE: This file must not be changed.

#ifndef _LINK_LAYER_H_
#define _LINK_LAYER_H_


typedef enum
{
    LlTx,
    LlRx,
} LinkLayerRole;

typedef enum {
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    PAYLOAD,
    PAYLOAD_ESC,
    BCC2_OK,
    STOP
} StateMachine;

typedef struct
{
    char serialPort[50];
    LinkLayerRole role;
    int baudRate;
    int nRetransmissions;
    int timeout;
} LinkLayer;

// SIZE of maximum acceptable payload.
// Maximum number of bytes that application layer should send to link layer
#define MAX_PAYLOAD_SIZE 1000

// Definition of Macros to help our code readability and implementation

#define C_I0 0x00
#define C_I1 0x80
#define C_RR0 0xAA
#define C_RR1 0xAB
#define C_REJ0 0x54
#define C_REJ1 0x55
#define DISC 0x0B


#define FLAG 0x7E
#define ESC 0x7D
#define OCT_RPL_FLAG 0x5E
#define OCT_RPL_ESC 0x5D

#define ADDR_SSAR 0x03
#define ADDR_SRAS 0x01

#define CNTRL_SET 0x03
#define CNTRL_UA 0x07


// MISC
#define FALSE 0
#define TRUE 1


void alarmHandler(int signal);



// Open a connection using the "port" parameters defined in struct linkLayer.
// Return "1" on success or "-1" on error.
int llopen(LinkLayer connectionParameters);

// Send data in buf with size bufSize.
// Return number of chars written, or "-1" on error.
int llwrite(const unsigned char *buf, int bufSize);

// Receive data in packet.
// Return number of chars read, or "-1" on error.
int llread(unsigned char *packet);

// Close previously opened connection.
// if showStatistics == TRUE, link layer should print statistics in the console on close.
// Return "1" on success or "-1" on error.
int llclose(int showStatistics);

unsigned char frame_control_check();

#endif // _LINK_LAYER_H_
