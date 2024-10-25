#include "serial_port.h"
#include "link_layer.h"
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

unsigned int trama_transmiter = 0;
unsigned int trama_receiver = 1;
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
            escrever_frame(ADDR_SSAR, CNTRL_SET);
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
        
        escrever_frame(ADDR_SRAS,CNTRL_UA);

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
    unsigned char bcc2 = buf[0];
    buffer[0] = FLAG;
    buffer[1] = ADDR_SSAR;
    buffer[2] = NTRAMA(trama_transmiter);
    buffer[3] = buffer[1] ^ buffer[2];

    for(int i =1; i< bufSize; i++){
        bcc2 ^= buf[i];
    }

    int w = 4;
    for (int i = 0; i < bufSize; i++)
    {
        if (buf[i] == FLAG)
        {
            buffer[w++] = ESC;
            buffer[w++] = OCT_RPL_FLAG; // Aqui transformamos a flag e o esc em 2 octetos, na função read temos de fazer o contrário para preservar dados
            printf("Repus A FLAG");
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

    while (tentativa_atual < transmissoes)
    {

        alarmEnabled = FALSE;
        alarm(intervalo);
        aceite = 0;
        rejeitado = 0;
        while (alarmEnabled == FALSE && aceite==0 && !rejeitado)
        {
            if (writeBytesSerialPort(buffer, w) != w)
            {
                return -1;
            }

            unsigned char campoC = frame_control_check();


            if (campoC == C_RR0 || campoC == C_RR1)
            {
                aceite = 1;
                trama_transmiter = (trama_transmiter +1) %2;
            }
            else if (campoC == C_REJ0 || campoC == C_REJ1)
            {
                rejeitado = 1;
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
    }
    if (aceite == 1)
    {
        return size_frame;
    }
    else
    {
        //llclose(FALSE);
        return -1;
    }
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    unsigned char buffer;
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
                    
                    unsigned char first_byte_of_payload = 0;
                    for (int w = 0; w < indice; w++)
                    {
                        first_byte_of_payload ^= packet[w];
                    }

                    if (bcc2 == first_byte_of_payload)
                    {
                        estado = STOP;
                        if(NTRAMA(trama_receiver)!=campoC){
                            if(trama_receiver == 0){
                                escrever_frame(ADDR_SSAR, C_RR0);
                                trama_receiver = (trama_receiver +1) %2;
                                printf("Enviei um receiver ready1\n");
                                return indice;
                            }else{
                                escrever_frame(ADDR_SSAR, C_RR1);
                                trama_receiver = (trama_receiver +1) %2;
                                printf("Enviei um receiver ready2\n");
                                return indice;
                            }
                        }else{
                            if(trama_receiver == 0){
                                escrever_frame(ADDR_SSAR, C_RR0);
                                trama_receiver = (trama_receiver +1) %2;
                                printf("Enviei um receiver ready, REPETIDA1 \n");
                                return 0;
                            }else{
                                escrever_frame(ADDR_SSAR, C_RR1);
                                trama_receiver = (trama_receiver +1) %2;
                                printf("Enviei um receiver ready, REPETIDA2 \n");
                                return 0;
                            }
                        }
                    }
                    else
                    {
                        if(NTRAMA(trama_receiver)!=campoC){
                            if(trama_receiver == 0){
                                escrever_frame(ADDR_SSAR, C_REJ0);
                                trama_receiver = (trama_receiver +1) %2;
                                printf("Enviei um receiver ready3\n");
                                return -1;
                            }else{
                                escrever_frame(ADDR_SSAR, C_REJ1);
                                trama_receiver = (trama_receiver +1) %2;
                                printf("Enviei um receiver ready4\n");
                                return -1;
                            }
                        }else{
                            if(trama_receiver == 0){
                                escrever_frame(ADDR_SSAR, C_RR0);
                                trama_receiver = (trama_receiver +1) %2;
                                printf("Enviei um receiver ready, REPETIDA3 \n");
                                return 0;
                            }else{
                                escrever_frame(ADDR_SSAR, C_RR1);
                                trama_receiver = (trama_receiver +1) %2;
                                printf("Enviei um receiver ready, REPETIDA4 \n");
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
                    packet[indice++] = buffer;
                }
                break;


            case PAYLOAD_ESC:
                estado = PAYLOAD;
                unsigned char identificador = packet[1];
                printf("stufing no packet: %d\n", identificador);
                if(buffer == 0x5E){
                    packet[indice++] = FLAG;
                }else if(buffer==0X5D){
                    packet[indice++] = ESC;
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
int llclose(int showStatistics,LinkLayer connectionParameters)
{
    StateMachine estado = START;

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
           
            escrever_frame(ADDR_SSAR, DISC);
            alarm(intervalo);
            alarmEnabled = FALSE;
            while (alarmEnabled == FALSE && estado != STOP)
            {
                if (readByteSerialPort(&buffer) > 0)
                {
                    if (estado == START)
                {
                    printf("Estado: START\n");
                    if (buffer == FLAG)
                    {
                        estado = FLAG_RCV;
                    }
                }
                else if (estado == FLAG_RCV)
                {
                    printf("Estado: FLAG_RCV\n");
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
                    printf("Estado: A_RCV\n");
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
                    printf("Estado: C_RCV\n");
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
                    printf("Estado: BCC_OK\n");
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
        escrever_frame(ADDR_SSAR, CNTRL_UA);
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
                    printf("Estado: START\n");
                    if (buffer == FLAG)
                    {
                        estado = FLAG_RCV;
                    }
                }
                else if (estado == FLAG_RCV)
                {
                    printf("Estado: FLAG_RCV\n");
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
                    printf("Estado: A_RCV\n");
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
                    printf("Estado: C_RCV\n");
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
                    printf("Estado: BCC_OK\n");
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
        
        escrever_frame(ADDR_SRAS,DISC);

        break;
    }

    default:
    {
        return -1;
        break;
    }
    }
    printf("Finished successfully\n");
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
                if (byte == FLAG)
                {
                    estado = FLAG_RCV;
                }
            }
            else if (estado == FLAG_RCV)
            {
                printf("Cheguei ao FLAG_RCV\n");

                if (byte == ADDR_SSAR)
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

                if (byte == FLAG)
                {
                    estado = FLAG_RCV;
                }
                else if (byte == C_RR0 || byte == C_RR1 || byte == C_REJ0 || byte == C_REJ1)
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
                if (byte == (ADDR_SSAR ^ campoC))
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

int escrever_frame(unsigned char address_field, unsigned char control_field){
    unsigned char buffer[5] = {FLAG, address_field, control_field, address_field^control_field, FLAG};
    return writeBytesSerialPort(buffer,5);
}