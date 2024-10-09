// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include <signal.h>
#include <stdio.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

int alarmCount = 0;
int alarmEnabled = FALSE;
int transmissoes = 0;
int intervalo = 0;

void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    StateMachine estado = START;
    if (openSerialPort(connectionParameters.serialPort,
                       connectionParameters.baudRate) < 0)
    {
        return -1;
    }

    LinkLayerRole r = connectionParameters.role;
    unsigned char *buffer;
    transmissoes = connectionParameters.nRetransmissions;
    int retransmitions = connectionParameters.nRetransmissions;
    intervalo = connectionParameters.timeout;
    switch (r)
    {
    case LlTx:
    {
        (void)signal(SIGALRM, alarmHandler);
        while (retransmitions > 0 && estado != STOP)
        {
            unsigned char trama[5] = {FLAG, ADDR_SSAR, CNTRL_SET, ADDR_SSAR ^ CNTRL_SET, FLAG};
            if (writeBytesSerialPort(trama, 5) != 0)
            {
                return -1;
            }
            alarm(intervalo);
            alarmEnabled = FALSE;
            while (alarmEnabled == FALSE && estado != STOP)
            {
                if (readByteSerialPort(buffer) != 0)
                {
                    if (estado == START)
                    {
                        if (buffer == FLAG)
                        {
                            estado = FLAG_RCV;
                        }
                    }
                    else if (estado == FLAG_RCV)
                    {
                        if (buffer == ADDR_SSAR)
                        {
                            estado = A_RCV;
                        }
                        else if (buffer != FLAG)
                        {
                            estado = START;
                        }
                    }
                    else if (estado == A_RCV)
                    {
                        if (buffer == FLAG)
                        {
                            estado = FLAG_RCV;
                        }
                        else if (buffer == CNTRL_SET)
                        {
                            estado = C_RCV;
                        }
                        else
                        {
                            estado = START;
                        }
                    }
                    else if (estado == C_RCV)
                    {
                        if (buffer == ADDR_SSAR ^ CNTRL_SET)
                        {
                            estado = BCC_OK;
                        }
                        else if (buffer == FLAG)
                        {
                            estado = FLAG_RCV;
                        }
                        else
                        {
                            estado = START;
                        }
                    }
                    else if (estado == BCC_OK)
                    {
                        if (buffer == FLAG)
                        {
                            estado = STOP;
                        }
                        else
                        {
                            estado = START;
                        }
                    }
                }
                retransmitions--;
            }
        }
        if (estado != STOP)
        {
            return -1;
        }
        break;
    }

    case LlRx:
    {
        while (estado != STOP)
        {
            if (readByteSerialPort(buffer) != 0)
            {
                if (estado == START)
                {
                    if (buffer == FLAG)
                    {
                        estado = FLAG_RCV;
                    }
                }
                else if (estado == FLAG_RCV)
                {
                    if (buffer == ADDR_SRAS)
                    {
                        estado = A_RCV;
                    }
                    else if (buffer != FLAG)
                    {
                        estado = START;
                    }
                }
                else if (estado == A_RCV)
                {
                    if (buffer == FLAG)
                    {
                        estado = FLAG_RCV;
                    }
                    else if (buffer == CNTRL_UA)
                    {
                        estado = C_RCV;
                    }
                    else
                    {
                        estado = START;
                    }
                }
                else if (estado == C_RCV)
                {
                    if (buffer == ADDR_SRAS ^ CNTRL_UA)
                    {
                        estado = BCC_OK;
                    }
                    else if (buffer == FLAG)
                    {
                        estado = FLAG_RCV;
                    }
                    else
                    {
                        estado = START;
                    }
                }
                else if (estado == BCC_OK)
                {
                    if (buffer == FLAG)
                    {
                        estado = STOP;
                    }
                    else
                    {
                        estado = START;
                    }
                }
            }
        }

        unsigned char trama[5] = {FLAG, ADDR_SRAS, CNTRL_UA, ADDR_SRAS ^ CNTRL_UA, FLAG};
        return writeBytesSerialPort(trama, 5);
        break;
    }

    default:
    {

        return -1;
        break;
    }
    }
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    int tentativa_atual = 0;
    int rejeitado = 0;
    int aceite = 0;
    int size_frame = bufSize + 6;
    unsigned char buffer[MAX_PAYLOAD_SIZE * 3];
    buffer[0] = FLAG;
    buffer[1] = ADDR_SSAR;
    buffer[2] = C_I0;
    buffer[3] = buffer[1] ^ buffer[2];

    unsigned int w = 4;
    for (unsigned int i = 0; i < bufSize; i++)
    {
        if (buf[i] == FLAG)
        {
            buffer[w++] = ESC;
            buffer[w++] = OCT_RPL_FLAG;
        }
        else if (buf[i] == ESC)
        {
            buffer[w++] = ESC;
            buffer[w++] = OCT_RPL_ESC;
        }
        else
        {
            buffer[w++] = buf[i];
        }
    }
    unsigned char bcc2 = buf[0];
    for (unsigned int i = 1; i < bufSize; i++)
    {
        bcc2 ^= buf[i];
    }

    buffer[w++] = bcc2;
    buffer[w++] = FLAG;

    while (tentativa_atual < transmissoes)
    {
        alarm(intervalo);
        alarmEnabled = FALSE;
        while (alarmEnabled == FALSE && !aceite && !rejeitado)
        {
            if (writeBytesSerialPort(buffer, w) != w)
            {
                return -1;
            }
            unsigned char byte;
            if (readByteSerialPort(&byte) != 0)
            {
                return -1;
            }
            unsigned char campoC = frame_control_check();
            if (campoC == C_RR0 || campoC == C_RR1)
            {
                aceite = 1;
            }
            else if (campoC == C_REJ0 || campoC == C_REJ1)
            {
                rejeitado = 1;
            }
        }
        if (aceite == 1)
        {
            break;
        }
        tentativa_atual++;
    }
    if (tentativa_atual == transmissoes)
    {
        return -1;
    }
    if (aceite == 1)
    {
        return bufSize;
    }
    else
    {
        // É SUPOSTO FECHAR A PORTA SE DER ERRO DEPOIS DE MUITAS TENTATIVAS?
        return -1;
    }
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    //PEDIR AJUDA AO PROFESSOR

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    StateMachine estado = START;
    unsigned char buffer;
    (void)signal(SIGALRM, alarmHandler);
    int retransmitions = transmissoes;
    unsigned char trama[5] = {FLAG, ADDR_SSAR, DISC, ADDR_SSAR ^ DISC, FLAG};
    if (writeBytesSerialPort(trama, 5) != 0)
    {
        return -1;
    }
    alarm(intervalo);
    alarmEnabled = FALSE;
    while (alarmEnabled == FALSE && estado != STOP)
    {
        if (readByteSerialPort(buffer) != 0)
        {
            if (estado == START)
            {
                if (buffer == FLAG)
                {
                    estado = FLAG_RCV;
                }
            }
            else if (estado == FLAG_RCV)
            {
                if (buffer == ADDR_SSAR)
                {
                    estado = A_RCV;
                }
                else if (buffer != FLAG)
                {
                    estado = START;
                }
            }
            else if (estado == A_RCV)
            {
                if (buffer == FLAG)
                {
                    estado = FLAG_RCV;
                }
                else if (buffer == CNTRL_SET)
                {
                    estado = C_RCV;
                }
                else
                {
                    estado = START;
                }
            }
            else if (estado == C_RCV)
            {
                if (buffer == ADDR_SSAR ^ CNTRL_SET)
                {
                    estado = BCC_OK;
                }
                else if (buffer == FLAG)
                {
                    estado = FLAG_RCV;
                }
                else
                {
                    estado = START;
                }
            }
            else if (estado == BCC_OK)
            {
                if (buffer == FLAG)
                {
                    estado = STOP;
                }
                else
                {
                    estado = START;
                }
            }
        }
        retransmitions--;
    }
    if (estado != STOP)
    {
        return -1;
    }
    unsigned char trama2[5] = {FLAG, ADDR_SSAR, CNTRL_UA, ADDR_SSAR ^ CNTRL_UA, FLAG};
    if (writeBytesSerialPort(trama2, 5) != 0)
    {
        return -1;
    }
    return closeSerialPort();
}

unsigned char frame_control_check()
{
    unsigned char byte = 0;
    unsigned char campoC;
    StateMachine estado = START;

    while (estado != STOP)
    {
        if (readByteSerialPort(&byte) != 0)
        {
            if (estado == START)
            {
                if (byte == FLAG)
                {
                    estado = FLAG_RCV;
                }
            }
            else if (estado == FLAG_RCV)
            {
                if (byte == ADDR_SRAS)
                {
                    estado = A_RCV;
                }
                else if (byte != FLAG)
                {
                    estado = START;
                }
            }
            else if (estado == A_RCV)
            {
                if (byte == FLAG)
                {
                    estado = FLAG_RCV;
                }
                else if (byte == C_RR0 || byte == C_RR1 || byte == C_REJ0 || byte == C_REJ1 || byte == DISC)
                {
                    estado = C_RCV;
                    campoC = byte;
                }
                else
                {
                    estado = START;
                }
            }
            else if (estado == C_RCV)
            {
                if (byte == ADDR_SRAS ^ campoC)
                {
                    estado = BCC_OK;
                }
                else if (byte == FLAG)
                {
                    estado = FLAG_RCV;
                }
                else
                {
                    estado = START;
                }
            }
            else if (estado == BCC_OK)
            {
                if (byte == FLAG)
                {
                    estado = STOP;
                }
                else
                {
                    estado = START;
                }
            }
        }
    }
    return campoC;
}