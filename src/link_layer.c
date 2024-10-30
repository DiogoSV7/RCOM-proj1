#include "serial_port.h"
#include "link_layer.h"
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>


// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

unsigned int trama_transmiter = 0;
unsigned int trama_receiver = 1;
int alarmCount = 0;
volatile int alarmEnabled = FALSE;
int transmissoes = 0;
int intervalo = 0;
extern int fd;
char *serialPort;
int baudrate;
LinkLayer layerzinha;

LinkLayerStats stats = {0};

// Se bcc1 estiver errado, não devo fazer nada, significa que o cabeçalho está errado e tenho de deitar tudo fora, esperar que do lado do transmissor de timeout e esperar que envie outra vez

// Se bcc2 falhar sabe que o cabeçalho está certo, mas algum dos bytes dos dados está errado, poderia simplesmente descartar e enviar tudo de novo, mas como sabe que o cabeçalho está bem pode mandar uma trama de REJ(0) e dizer que recebeu uma trama errada e assim o transmissor envia logo e não espera pelo timeout, o que acelera o processo
// Mecanismo do bcc2 não é necessário, porém dá melhor nota

void alarmHandler(int signal)
{
    alarmEnabled = TRUE;
    printf("Alarm has been triggered!\n");
    alarmCount++;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    StateMachine estado = START;
    int open = openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate);

    if (open < 0)
    {
        return -1;
    }
    layerzinha = connectionParameters;
    LinkLayerRole r = connectionParameters.role;
    unsigned char buffer;
    transmissoes = connectionParameters.nRetransmissions;
    int retransmitions = connectionParameters.nRetransmissions;
    intervalo = connectionParameters.timeout;
    serialPort = connectionParameters.serialPort;
    baudrate = connectionParameters.baudRate;

    stats.startTime = time(NULL);
    switch (r)
    {
    case LlTx:
    {
        struct sigaction act = {0};
        act.sa_handler = &alarmHandler;
        if (sigaction(SIGALRM, &act, NULL) == -1)
        {
            perror("sigaction");
            exit(EXIT_FAILURE);
        }
        while (retransmitions > 0 && estado != STOP)
        {
            unsigned char trama[5] = {FLAG, ADDR_SSAR, CNTRL_SET, ADDR_SSAR ^ CNTRL_SET, FLAG};
            writeBytesSerialPort(trama, 5);
            stats.sentFrames++;
            printf("Tried to establish connection with the Receiver!\n");
            alarm(intervalo);
            alarmEnabled = FALSE;

            while (alarmEnabled == FALSE && estado != STOP)
            {
                if (readByteSerialPort(&buffer) > 0)
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
                        if (buffer == (ADDR_SRAS ^ CNTRL_UA))
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
            retransmitions--;
        }
        if (estado != STOP)
        {
            printf("Error in the connection!\n");
            return -1;
        }
        stats.receivedFrames++;
        printf("Connection has been established with the Receiver!\n");
        break;
    }

    case LlRx:
    {
        while (estado != STOP)
        {
            if (readByteSerialPort(&buffer) > 0)
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
                    if (buffer == (ADDR_SSAR ^ CNTRL_SET))
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
                    fflush(stdout);
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
        writeBytesSerialPort(trama, 5);
        stats.sentFrames++;
        printf("Connection has been established with the Transmiter\n");
        break;
    }
    stats.endTime = time(NULL);
    stats.totalTime += difftime(stats.endTime, stats.startTime);
    default:
    {
        return -1;
        printf("Error in the connection!\n");
        break;
    }
    }

    return open;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    stats.startTime = time(NULL);
    int tentativa_atual = 0;
    int rejeitado = 0;
    int aceite = 0;
    int size_frame = bufSize + 6;
    unsigned char *buffer = (unsigned char *)malloc(size_frame);
    unsigned char bcc2 = buf[0];
    buffer[0] = FLAG;
    buffer[1] = ADDR_SSAR;
    buffer[2] = NTRAMA(trama_transmiter);
    buffer[3] = buffer[1] ^ buffer[2];

    for (int i = 1; i < bufSize; i++)
    {
        bcc2 ^= buf[i];
    }

    int w = 4;
    for (int i = 0; i < bufSize; i++)
    {
        if (buf[i] == FLAG)
        {
            buffer = realloc(buffer, size_frame);
            buffer[w++] = ESC;
            buffer[w++] = OCT_RPL_FLAG; // Aqui transformamos a flag e o esc em 2 octetos, na função read temos de fazer o contrário para preservar dados
        }
        else if (buf[i] == ESC)
        {
            buffer = realloc(buffer, size_frame);
            buffer[w++] = ESC;
            buffer[w++] = OCT_RPL_ESC;
        }
        else
        {
            buffer[w++] = buf[i];
        }
    }
    if (bcc2 == FLAG)
    {
        buffer = realloc(buffer, size_frame);
        buffer[w++] = ESC;
        buffer[w++] = OCT_RPL_FLAG;
    }
    else if (bcc2 == ESC)
    {
        buffer = realloc(buffer, size_frame);
        buffer[w++] = ESC;
        buffer[w++] = OCT_RPL_ESC;
    }
    else
    {
        buffer[w++] = bcc2;
    }

    buffer[w++] = FLAG;

    while (tentativa_atual < transmissoes)
    {

        alarmEnabled = FALSE;
        alarm(intervalo);
        aceite = 0;
        rejeitado = 0;
        while (alarmEnabled == FALSE && aceite == 0 && !rejeitado)
        {
            if (writeBytesSerialPort(buffer, w) < 0)
            {
                printf("Error in writting bytes!\n");
                stats.errors++;
                exit(-1);
            }

            unsigned char campoC = frame_control_check();

            if (campoC == C_RR0 || campoC == C_RR1)
            {
                aceite = 1;
                stats.sentFrames++;
                trama_transmiter = (trama_transmiter + 1) % 2;
            }
            else if (campoC == C_REJ0 || campoC == C_REJ1)
            {
                rejeitado = 1;
                tentativa_atual = 0;
                stats.errors++;
            }
            else
            {
                continue;
            }
        }
        if (aceite == 1)
        {
            break;
        }
        tentativa_atual++;
        stats.retransmissions++;
    }
    free(buffer);
    stats.endTime = time(NULL);
    stats.totalTime += difftime(stats.endTime, stats.startTime);
    if (aceite == 1)
    {
        
        printf("Wrote frame to Receiver, waiting for Receiver´s response...\n");
        return size_frame;
    }
    else
    {
        printf("Error in writing bytes");
        return -1;
    }
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    struct sigaction act = {0};
    act.sa_handler = &alarmHandler;
    if (sigaction(SIGALRM, &act, NULL) == -1)
    {
        perror("sigaction");
        exit(EXIT_FAILURE);
    }
    int retries = 0;
    unsigned char buffer;
    int indice_atual = 0;
    StateMachine estado = START;
    unsigned char campoC;

    stats.startTime = time(NULL);
    
    while (estado != STOP && retries < transmissoes)
    {
        alarm(intervalo);
        alarmEnabled = FALSE;
        while (alarmEnabled == FALSE && estado != STOP)
        {

            int r = readByteSerialPort(&buffer);
            if (r > 0)
            {
                switch (estado)
                {
                case START:
                    if (buffer == FLAG)
                    {
                        estado = FLAG_RCV;
                    }
                    break;

                case FLAG_RCV:
                    if (buffer == ADDR_SSAR)
                    {
                        estado = A_RCV;
                    }
                    else
                    {
                        estado = START;
                    }
                    break;

                case A_RCV:
                    if (buffer == FLAG)
                    {
                        estado = FLAG_RCV;
                    }
                    else if (buffer == C_I0 || buffer == C_I1)
                    {
                        estado = C_RCV;
                        campoC = buffer;
                    }
                    else if (buffer == DISC)
                    {
                        unsigned char trama[5] = {FLAG, ADDR_SRAS, DISC, ADDR_SRAS ^ DISC, FLAG};
                        writeBytesSerialPort(trama, 5);
                        stats.sentFrames++;
                        return 0;
                    }
                    else
                    {
                        estado = START;
                    }
                    break;

                case C_RCV:
                    if (buffer == (ADDR_SSAR ^ campoC))
                    {
                        estado = PAYLOAD;
                    }
                    else if (buffer == FLAG)
                    {
                        estado = FLAG_RCV;
                    }
                    else
                    {
                        estado = START;
                    }
                    break;

                case PAYLOAD:
                    stats.receivedFrames++;
                    if (buffer == FLAG)
                    {
                        unsigned char bcc2 = packet[indice_atual - 1];
                        indice_atual--;

                        unsigned char first_byte_of_payload = 0;
                        for (int w = 0; w < indice_atual; w++)
                        {
                            first_byte_of_payload ^= packet[w];
                        }

                        if (bcc2 == first_byte_of_payload)
                        {
                            estado = STOP;
                            if (NTRAMA(trama_receiver) != campoC)
                            {
                                if (trama_receiver == 0)
                                {
                                    unsigned char trama[5] = {FLAG, ADDR_SSAR, C_RR0, ADDR_SSAR ^ C_RR0, FLAG};
                                    writeBytesSerialPort(trama, 5);
                                    trama_receiver = (trama_receiver + 1) % 2;
                                    printf("Received the correct frame, asked to send the next one!\n");
                                    return indice_atual;
                                }
                                else
                                {
                                    unsigned char trama[5] = {FLAG, ADDR_SSAR, C_RR1, ADDR_SSAR ^ C_RR1, FLAG};
                                    writeBytesSerialPort(trama, 5);
                                    trama_receiver = (trama_receiver + 1) % 2;
                                    printf("Received the correct frame, asked to send the next one!\n");
                                    return indice_atual;
                                }
                            }
                            else
                            {
                                if (trama_receiver == 0)
                                {
                                    unsigned char trama[5] = {FLAG, ADDR_SSAR, C_RR0, ADDR_SSAR ^ C_RR0, FLAG};
                                    writeBytesSerialPort(trama, 5);
                                    trama_receiver = (trama_receiver + 1) % 2;
                                    printf("Received duplicate correct frame, asked to send the next one!\n");
                                    return 0;
                                }
                                else
                                {
                                    unsigned char trama[5] = {FLAG, ADDR_SSAR, C_RR1, ADDR_SSAR ^ C_RR1, FLAG};
                                    writeBytesSerialPort(trama, 5);
                                    trama_receiver = (trama_receiver + 1) % 2;
                                    printf("Received duplicate correct frame, asked to send the next one!\n");
                                    return 0;
                                }
                            }
                        }
                        else
                        {
                            if (NTRAMA(trama_receiver) != campoC)
                            {
                                if (trama_receiver == 0)
                                {
                                    unsigned char trama[5] = {FLAG, ADDR_SSAR, C_REJ0, ADDR_SSAR ^ C_REJ0, FLAG};
                                    writeBytesSerialPort(trama, 5);
                                    stats.errors++;
                                    printf("Rejected frame with the ID 0.\n");
                                    return -1;
                                }
                                else
                                {
                                    unsigned char trama[5] = {FLAG, ADDR_SSAR, C_REJ1, ADDR_SSAR ^ C_REJ1, FLAG};
                                    writeBytesSerialPort(trama, 5);
                                    stats.errors++;
                                    printf("Rejected frame with the ID 1.\n");
                                    return -1;
                                }
                            }
                            else
                            {
                                if (trama_receiver == 0)
                                {
                                    unsigned char trama[5] = {FLAG, ADDR_SSAR, C_RR0, ADDR_SSAR ^ C_RR0, FLAG};
                                    writeBytesSerialPort(trama, 5);
                                    trama_receiver = (trama_receiver + 1) % 2;
                                    printf("Received wrong frame duplicated with the ID 0.\n");
                                    return 0;
                                }
                                else
                                {
                                    unsigned char trama[5] = {FLAG, ADDR_SSAR, C_RR1, ADDR_SSAR ^ C_RR1, FLAG};
                                    writeBytesSerialPort(trama, 5);
                                    trama_receiver = (trama_receiver + 1) % 2;
                                    printf("Received wrong frame duplicated with the ID 1 .\n");
                                    return 0;
                                }
                            }
                        }
                    }
                    else if (buffer == ESC)
                    {
                        estado = PAYLOAD_ESC;
                    }
                    else
                    {
                        packet[indice_atual++] = buffer;
                    }
                    break;

                case PAYLOAD_ESC:
                    estado = PAYLOAD;
                    printf("Destufing the packet number: %d\n", packet[1]);
                    if (buffer == OCT_RPL_FLAG)
                    {
                        packet[indice_atual++] = FLAG;
                    }
                    else if (buffer == OCT_RPL_ESC)
                    {
                        packet[indice_atual++] = ESC;
                    }
                    break;

                default:
                    break;
                }
            }
            else if (r == -1)
            {
                stats.errors++;
                perror("Connection lost, attempting to reconnect...");
                fd = llopen(layerzinha);
                if (fd < 0)
                {
                    retries++;
                    printf("Reconnect failed, retrying...\n");
                }
                else
                {
                    printf("Reconnected successfully, resuming...\n");
                    return 0;
                }
            }
        }
    }
    stats.endTime = time(NULL);
    stats.totalTime += difftime(stats.endTime, stats.startTime);
    return -1;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics, LinkLayer connectionParameters)
{
    StateMachine estado = START;

    LinkLayerRole connection_role = connectionParameters.role;

    unsigned char buffer;
    transmissoes = connectionParameters.nRetransmissions;
    int retransmitions = connectionParameters.nRetransmissions;
    intervalo = connectionParameters.timeout;
    stats.startTime = time(NULL);
    switch (connection_role)
    {
    case LlTx:
    {
        struct sigaction act = {0};
        act.sa_handler = &alarmHandler;
        if (sigaction(SIGALRM, &act, NULL) == -1)
        {
            perror("sigaction");
            exit(EXIT_FAILURE);
        }
        while (retransmitions > 0 && estado != STOP)
        {
            unsigned char trama[5] = {FLAG, ADDR_SSAR, DISC, ADDR_SSAR ^ DISC, FLAG};
            writeBytesSerialPort(trama, 5);
            stats.sentFrames++;
            printf("Sent disconnect frame to the receiver!\n");
            alarm(intervalo);
            alarmEnabled = FALSE;
            while (alarmEnabled == FALSE && estado != STOP)
            {
                if (readByteSerialPort(&buffer) > 0)
                {
                    stats.receivedFrames++;
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
                        else if (buffer == DISC)
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
                        if (buffer == (ADDR_SRAS ^ DISC))
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
            retransmitions--;
        }
        if (estado != STOP)
        {
            return -1;
        }
        unsigned char trama[5] = {FLAG, ADDR_SSAR, CNTRL_UA, ADDR_SSAR ^ CNTRL_UA, FLAG};
        writeBytesSerialPort(trama, 5);
        stats.sentFrames++;
        printf("Received the correct disconnect response from the Receiver.\n");
        break;
    }

    case LlRx:
    {
        while (estado != STOP)
        {
            if (readByteSerialPort(&buffer) > 0)
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
                    else if (buffer == DISC)
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
                    if (buffer == (ADDR_SSAR ^ DISC))
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
        stats.endTime = time(NULL);
        stats.totalTime += difftime(stats.endTime, stats.startTime);
        unsigned char trama[5] = {FLAG, ADDR_SRAS, DISC, ADDR_SRAS ^ DISC, FLAG};
        writeBytesSerialPort(trama, 5);
        stats.sentFrames++;
        printf("Sent the confirmation of the disconnect to the Transmiter!\n");
        break;
    }

    default:
    {
        return -1;
        break;
    }
    }
    printf("Finished the transfer successfully!\n");
    printf("\n");
    printf("=== Communication Statistics ===\n");
    printf("Sent Frames: %d\n", stats.sentFrames);
    printf("Received Frames: %d\n", stats.receivedFrames);
    printf("Retransmissions: %d\n", stats.retransmissions);
    printf("Errors: %d\n", stats.errors);
    printf("Total Time: %.2f seconds\n", stats.totalTime);
    printf("\n");
    return closeSerialPort();
}

unsigned char frame_control_check()
{
    unsigned char buffer = 0;
    unsigned char campoC = 0;
    StateMachine estado = START;
    int retries = 0;
    while (estado != STOP && alarmEnabled == FALSE)
    {   
        int r = readByteSerialPort(&buffer);
        if (r > 0)
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
                else if (buffer == C_RR0 || buffer == C_RR1 || buffer == C_REJ0 || buffer == C_REJ1 || buffer == DISC)
                {
                    estado = C_RCV;
                    campoC = buffer;
                }
                else
                {
                    estado = START;
                }
            }
            else if (estado == C_RCV)
            {
                if (buffer == (ADDR_SSAR ^ campoC))
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
        else if (r == -1)
        {
            stats.errors++;
            perror("Write failed due to connection loss");
            printf("Attempting to reconnect...\n");

            fd = llopen(layerzinha);
            if (fd < 0)
            {
                retries++;
                printf("Reconnect failed, retrying...\n");
            }
            else
            {
                printf("Reconnected successfully, resuming...\n");
                retries = 0;
                return 0;
            }
        }
    }
    return campoC;
}