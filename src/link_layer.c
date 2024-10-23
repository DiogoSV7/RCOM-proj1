#include "serial_port.h"
#include "link_layer.h"
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

unsigned char trama_transmiter = 0x00;
unsigned char trama_receiver = 0x01;
int alarmCount = 0;
volatile int alarmEnabled = FALSE;
int transmissoes = 0;
int intervalo = 0;

// Se bcc1 estiver errado, não devo fazer nada, significa que o cabeçalho está errado e tenho de deitar tudo fora, esperar que do lado do transmissor de timeout e esperar que envie outra vez

// Se bcc2 falhar sabe que o cabeçalho está certo, mas algum dos bytes dos dados está errado, poderia simplesmente descartar e enviar tudo de novo, mas como sabe que o cabeçalho está bem pode mandar uma trama de REJ(0) e dizer que recebeu uma trama errada e assim o transmissor envia logo e não espera pelo timeout, o que acelera o processo
// Mecanismo do bcc2 não é necessário, porém dá melhor nota

void alarmHandler(int signal)
{
    alarmEnabled = TRUE;
    printf("Alarm triggered\n");
    alarmCount++;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    printf("i");
    StateMachine estado = START;
    int var = openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate);

    if (var < 0)
    {
        return -1;
    }

    LinkLayerRole r = connectionParameters.role;
    unsigned char buffer;
    transmissoes = connectionParameters.nRetransmissions;
    int retransmitions = connectionParameters.nRetransmissions;
    intervalo = connectionParameters.timeout;

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
            if (writeBytesSerialPort(trama, 5) < 0)
            {
                return -1;
            }

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
            return -1;
        }
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
                    printf("Cheguei ap BCC\n");
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
        printf(" SENTA NA PICA DO PRETO\n ");
        fflush(stdout);
        if (writeBytesSerialPort(trama, 5) < 0)
        {
            return -1;
        }

        break;
    }

    default:
    {
        return -1;
        break;
    }
    }

    return var;
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
    if ((trama_transmiter & 0x01) == 0x00)
    {
        buffer[2] = C_I0;
    }
    else if ((trama_transmiter & 0x01) == 0x01)
    {
        buffer[2] = C_I1;
    }

    buffer[3] = buffer[1] ^ buffer[2];

    unsigned char bcc2 = buf[0];
    for (unsigned int i = 1; i < bufSize; i++) // bcc2 tem de ser calculado antes das transformação de stuffing
    {
        bcc2 ^= buf[i];
    }
    printf("Cheguei até ao BCC1");
    printf(" Received byte: 0x%02X\n", buffer);
    fflush(stdout);
    unsigned int w = 4;
    for (unsigned int i = 0; i < bufSize; i++)
    {
        if (buf[i] == FLAG)
        {
            buffer[w++] = ESC;
            buffer[w++] = OCT_RPL_FLAG; // Aqui transformamos a flag e o esc em 2 octetos, na função read temos de fazer o contrário para preservar dados
            printf("Repus o falog");
            fflush(stdout);
        }
        else if (buf[i] == ESC)
        {
            buffer[w++] = ESC;
            buffer[w++] = OCT_RPL_ESC;
            printf("Repus o ESC");
            fflush(stdout);
        }
        else
        {
            buffer[w++] = buf[i];
        }
    }

    //buffer[w++] = bcc2;
    if(bcc2 == FLAG){
        buffer[w++] = 0x7D;
        buffer[w++] = 0x5E;   
    }
    else if(bcc2 == ESC){
        buffer[w++] = 0x7D;
        buffer[w++] = 0x5D;   
    }
    else{
        buffer[w++] = bcc2;
    }

    buffer[w++] = FLAG;


    printf("Construi o buffer inteiro");
    printf(" Received byte: 0x%02X\n", buffer);
    fflush(stdout);
    while (tentativa_atual < transmissoes)
    {
        printf("Tentei transmitir");
        fflush(stdout);
        alarmEnabled = FALSE;
        alarm(intervalo);
        aceite = 0;
        rejeitado = 0;
        while (alarmEnabled == FALSE && !aceite && !rejeitado)
        {
            if (writeBytesSerialPort(buffer, w) != w)
            {
                printf("Escrevi nº errado de frames\n");
                fflush(stdout);
                return -1;
            }
            printf("Não tentei escrever nº errado\n");
            fflush(stdout);
            /*unsigned char byte;
            if (readByteSerialPort(&byte) > 0)
            {
                printf("Tentei ler da serial port e deu erro\n");
                fflush(stdout);
                return -1;
            }
            */

            printf("CHEGUEI ANTES DO CAMPO C");
            fflush(stdout);

            unsigned char campoC = frame_control_check();
            printf("CHEGUEI AO CAMPO C");
            printf(" Received byte: 0x%02X\n", campoC);
            fflush(stdout);
            if (campoC == C_RR0 || campoC == C_RR1)
            {
                printf("Comparei o campo C");
                fflush(stdout);
                //if (trama_transmiter == 1)
               // {
              //      trama_transmiter = 0;
              //  }
              //  else
               // {
               //     trama_transmiter = 1;
                //}
                if ((trama_transmiter & 0x01) == 0x00)
                    {
                        trama_transmiter = 0x01;
                    }
                    else if ((trama_transmiter & 0x01) == 0x01)
                    {
                        trama_transmiter = 0x00;
                    }
                
                aceite = 1;
                printf("trama transmiter= %i", trama_transmiter);
                printf("aceite= %i", aceite);
                fflush(stdout);
            }
            else if (campoC == C_REJ0 || campoC == C_REJ1)
            {
                rejeitado = 1;
                printf("REJtrama transmiter= %i", trama_transmiter);
                printf("REJaceite= %i", aceite);
                fflush(stdout);
            }
            else
            {
                continue;
            }
            printf("Comparei o campo CCCCCCC");
            fflush(stdout);
        }
        if (aceite == 1)
        {
            break;
        }
        tentativa_atual++;
    }
    if (aceite == 1)
    {
        return size_frame;
    }
    else
    {
        printf("deuMERDAAAA");
        llclose(FALSE);
        return -1;
    }
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    unsigned char buffer;
    unsigned char first_byte_of_payload;
    int indice = 0;
    StateMachine estado = START;
    unsigned char campoC;

    while (estado != STOP)
    {
        if (readByteSerialPort(&buffer) > 0)
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
                if (buffer == FLAG)
                {
                    unsigned char bcc2 = packet[indice - 1];
                    indice--;
                    
                    first_byte_of_payload = packet[0];
                    for (unsigned int w = 1; w < indice; w++)
                    {
                        first_byte_of_payload ^= packet[w];
                    }

                    if (bcc2 == first_byte_of_payload)
                    {
                        estado = STOP;
                        unsigned char trama[5] = {FLAG, ADDR_SRAS, (trama_receiver == 0x01 ? C_RR1 : C_RR0), ADDR_SRAS ^ (trama_receiver == 0x01 ? C_RR1 : C_RR0), FLAG};
                        if ((trama_receiver & 0x01) == 0x00)
                        {
                            trama_receiver = 0x01;
                        }
                        else if ((trama_receiver & 0x01) == 0x01)
                        {
                            trama_receiver = 0x00;
                        }
                        writeBytesSerialPort(trama, 5);

                        return indice;
                    }
                    else
                    {
                        unsigned char trama[5] = {FLAG, ADDR_SRAS, (trama_receiver == 0x01 ? C_REJ1 : C_REJ0), ADDR_SRAS ^ (trama_receiver == 0x01 ? C_REJ1 : C_REJ0), FLAG};
                        if ((trama_receiver & 0x01) == 0x00)
                        {
                            trama_receiver = 0x01;
                        }
                        else if ((trama_receiver & 0x01) == 0x01)
                        {
                            trama_receiver = 0x00;
                        }
                        writeBytesSerialPort(trama, 5);
                        return -1;
                    }
                }
                else if (buffer == ESC)
                {
                    estado = PAYLOAD_ESC;
                }
                else
                {
                    packet[indice++] = buffer;
                }
                break;


            case PAYLOAD_ESC:
                estado = PAYLOAD;
                if (buffer == FLAG || buffer == ESC)
                    {
                        packet[indice++] = buffer;
                    }
                    else
                    {
                        packet[indice++] = ESC;
                        packet[indice++] = buffer;
                    }

                break;

            default:
                // estado = START;
                break;
            }
            
        }
    }
    return -1;
}
////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    StateMachine estado = START;
    unsigned char buffer;
    struct sigaction act = {0};
    act.sa_handler = &alarmHandler;
    if (sigaction(SIGALRM, &act, NULL) == -1)
    {
        perror("sigaction");
        exit(EXIT_FAILURE);
    }
    int retransmitions = transmissoes;
    while (estado != STOP && retransmitions > 0)
    {
        unsigned char trama[5] = {FLAG, ADDR_SSAR, DISC, ADDR_SSAR ^ DISC, FLAG};
        if (writeBytesSerialPort(trama, 5) != 0)
        {
            return -1;
        }
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
                    else
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
    unsigned char campoC = 0;
    StateMachine estado = START;

    while (estado != STOP && alarmEnabled == FALSE)
    {
        if (readByteSerialPort(&byte) > 0 )
        {
            if (estado == START)
            {
                printf("Cheguei ap Start\n");

                printf(" Received byte: 0x%02X\n", byte);
                fflush(stdout);
                if (byte == FLAG)
                {
                    estado = FLAG_RCV;
                }
            }
            else if (estado == FLAG_RCV)
            {
                printf("Cheguei ao FLAG_RCV\n");
                fflush(stdout);
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
                printf("Cheguei ap A_RCV\n");
                printf(" Received byte: 0x%02X\n", byte);
                fflush(stdout);
                if (byte == FLAG)
                {
                    estado = FLAG_RCV;
                }
                else if (byte == C_RR0 || byte == C_RR1 || byte == C_REJ0 || byte == C_REJ1 || byte == CNTRL_SET)
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
                if (byte == (ADDR_SRAS ^ campoC))
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