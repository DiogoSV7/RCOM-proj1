// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include <signal.h>
#include <stdio.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

static int trama_transmiter = 0;
static int trama_receiver = 1;
int alarmCount = 0;
int alarmEnabled = FALSE;
int transmissoes = 0;
int intervalo = 0;

// Se bcc1 estiver errado, não devo fazer nada, significa que o cabeçalho está errado e tenho de deitar tudo fora, esperar que do lado do transmissor de timeout e esperar que envie outra vez

// Se bcc2 falhar sabe que o cabeçalho está certo, mas algum dos bytes dos dados está errado, poderia simplesmente descartar e enviar tudo de novo, mas como sabe que o cabeçalho está bem pode mandar uma trama de REJ(0) e dizer que recebeu uma trama errada e assim o transmissor envia logo e não espera pelo timeout, o que acelera o processo
// Mecanismo do bcc2 não é necessário, porém dá melhor nota

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
    if(trama_transmiter == 0){
        buffer[2] = C_I0;
    }else if(trama_transmiter == 1){
        buffer[2] = C_I1;
    }
        
    buffer[3] = buffer[1] ^ buffer[2];

    unsigned char bcc2 = buf[0];
    for (unsigned int i = 1; i < bufSize; i++) // bcc2 tem de ser calculado antes das transformação de stuffing
    {
        bcc2 ^= buf[i];
    }

    unsigned int w = 4;
    for (unsigned int i = 0; i < bufSize; i++)
    {
        if (buf[i] == FLAG)
        {
            buffer[w++] = ESC;
            buffer[w++] = OCT_RPL_FLAG; // Aqui transformamos a flag e o esc em 2 octetos, na função read temos de fazer o contrário para preservar dados
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
                if(trama_transmiter == 1){
                    trama_transmiter = 0;
                }else{
                    trama_transmiter = 1;
                }
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
        return -1;
    }
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    unsigned char *buffer;
    unsigned char first_byte_of_payload;
    int indice = 0;
    StateMachine estado = START;
    unsigned char campoC;
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
                    else if (buffer == C_I0 || buffer == C_I1)
                    {
                        estado = C_RCV;
                        campoC = buffer;
                    }
                    else if(buffer == DISC){
                        unsigned char trama[5] = {FLAG, ADDR_SRAS, DISC, ADDR_SRAS ^ DISC, FLAG};
                        return writeBytesSerialPort(trama, 5);
                        break;
                    }
                    else
                    {
                        estado = START;
                    }
                }
                else if (estado == C_RCV)
                {
                    if (buffer == ADDR_SSAR ^ campoC)
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
                    estado = PAYLOAD;
                }
                else if (estado == PAYLOAD)
                {
                    if (buffer == FLAG)
                    {
                        unsigned char bcc2 = packet[indice-1];
                        indice--;

                        first_byte_of_payload = packet[0];

                        for(unsigned int w = 1; w < indice; w++){
                            first_byte_of_payload ^= packet[w];
                        }
                        if(bcc2 == first_byte_of_payload){
                            estado = STOP;
                            if(trama_receiver == 1){
                                unsigned char trama[5] = {FLAG, ADDR_SRAS, C_I1, ADDR_SRAS ^ C_I1, FLAG};
                                if(trama_receiver == 1){
                                    trama_receiver =0;
                                }
                                else if(trama_receiver == 0){
                                    trama_receiver =1;
                                }
                                writeBytesSerialPort(trama, 5);
                                return indice;
                            }
                            else{
                                unsigned char trama[5] = {FLAG, ADDR_SRAS, C_I0, ADDR_SRAS ^ C_I0, FLAG};
                                if(trama_receiver == 1){
                                    trama_receiver =0;
                                }
                                else if(trama_receiver == 0){
                                    trama_receiver =1;
                                }
                                writeBytesSerialPort(trama, 5);
                                return indice;
                        }
                        }
                        else{
                            if(trama_receiver == 1){
                                unsigned char trama[5] = {FLAG, ADDR_SRAS, C_REJ1, ADDR_SRAS ^ C_REJ1, FLAG};
                                if(trama_receiver == 1){
                                    trama_receiver =0;
                                }
                                else if(trama_receiver == 0){
                                    trama_receiver =1;
                                }
                                writeBytesSerialPort(trama, 5);
                                return indice;
                            }
                            else{
                                unsigned char trama[5] = {FLAG, ADDR_SRAS, C_REJ0, ADDR_SRAS ^ C_REJ0, FLAG};
                                if(trama_receiver == 1){
                                    trama_receiver =0;
                                }
                                else if(trama_receiver == 0){
                                    trama_receiver =1;
                                }
                                writeBytesSerialPort(trama, 5);
                                return indice;
                        }
                        
                        }
                            
        
                    }
                    else
                    {
                        estado = START;
                    }
                }
            }
        }

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