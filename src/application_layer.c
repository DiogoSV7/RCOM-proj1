// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <string.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

unsigned char *getFileNameFromPacket(unsigned char *packet)
{
    unsigned char sizeLen = packet[2];
    unsigned char nameLen = packet[3 + sizeLen + 1];
    unsigned char *filename = (unsigned char *)malloc(nameLen);
    memcpy(filename, packet + 3 + sizeLen + 2, nameLen);
    return filename;
}

long int getFileSizeFromPacket(unsigned char *packet)
{
    unsigned char sizeLen = packet[2];
    unsigned char buf[sizeLen];
    long int fileSize = 0;
    memcpy(buf, packet + 3, sizeLen);
    for (int i = 0; i < sizeLen; i++)
    {
        fileSize |= (buf[sizeLen - 1 - i] << (8 * i));
    }
    return fileSize;
}

long int getFileSize(FILE *file)
{
    fseek(file, 0L, SEEK_END);
    long int size = ftell(file);
    fseek(file, 0, SEEK_SET);
    return size;
}

unsigned char *createControlPacket(const char *filename, long int fsize, unsigned int *length)
{
    int sizeLen = 0;
    long int sizeCopy = fsize;
    while (sizeCopy > 0)
    {
        sizeLen++;
        sizeCopy /= 255;
    }
    int nameLen = strlen(filename);
    *length = 3 + sizeLen + 2 + nameLen;
    unsigned char *packet = (unsigned char *)malloc(*length);
    packet[0] = 1;
    packet[1] = 0;
    packet[2] = sizeLen;
    for (int i = sizeLen + 2; i >= 3; i--)
    {
        packet[i] = (unsigned char)(fsize & 0XFF);
        fsize >>= 8;
    }
    int position = 2;
    position += sizeLen + 1;
    packet[position] = 1;
    position++;
    packet[position] = nameLen;
    position++;
    for (int j = 0; j < nameLen; j++)
    {
        packet[position + j] = filename[j];
    }
    return packet;
}

void createDataPacket(FILE *file, unsigned char *dPacket, int dSize, unsigned char id)
{
    dPacket[0] = 2;
    dPacket[1] = id;
    dPacket[2] = (dSize >> 8) & 0xFF;
    dPacket[3] = dSize & 0xFF;
    fread(dPacket + 4, 1, dSize, file);
}

void applicationLayer(const char *serialPort, const char *role, int baudRate, int nTries, int timeout, const char *filename)
{
    LinkLayer linklayer;
    strcpy(linklayer.serialPort, serialPort);
    linklayer.role = (strcmp(role, "rx") == 0) ? LlRx : LlTx;
    linklayer.nRetransmissions = nTries;
    linklayer.timeout = timeout;
    linklayer.baudRate = baudRate;
    int fd = llopen(linklayer);
    if (fd < 0)
    {
        perror("Failed to establish connection between transmitter and receiver\n");
        exit(-1);
    }
    else
    {
        printf("Successfully established connection between transmitter and receiver\n");
    }

    switch (linklayer.role)
    {
    case LlTx:
    {
        FILE *file = fopen(filename, "rb");
        if (file == NULL)
        {
            perror("Error opening source file");
            exit(EXIT_FAILURE);
        }
        unsigned int controlPacketlength;
        long int fsize = getFileSize(file);
        unsigned char *controlPacket = createControlPacket(filename, fsize, &controlPacketlength);

        if (llwrite(controlPacket, controlPacketlength) == -1)
        {
            perror("Failed to send start control packet");
            exit(EXIT_FAILURE);
        }
        else
        {
            printf("Successfully sent start control packet\n");
        }

        long int numberOfBytes = fsize;
        unsigned char id = 0;

        while (numberOfBytes > 0)
        {
            int dataSize = numberOfBytes > (long int)(MAX_PAYLOAD_SIZE - 4) ? (MAX_PAYLOAD_SIZE - 4) : numberOfBytes;
            int dataPacketSize = dataSize + 4;
            unsigned char *dataPacket = (unsigned char *)malloc(dataPacketSize);
            createDataPacket(file, dataPacket, dataSize, id);

            if (llwrite(dataPacket, dataPacketSize) == -1)
            {
                perror("Failed to send data packet\n");
                exit(EXIT_FAILURE);
            }
            numberOfBytes -= dataSize;
            id = (id + 1) % 255;
        }

        controlPacket[0] = 3;
        if (llwrite(controlPacket, controlPacketlength) == -1)
        {
            perror("Failed to send end control packet!\n");
            exit(EXIT_FAILURE);
        }
        else
        {
            printf("Successfully sent end control packet!\n");
        }
        llclose(fd, linklayer);
        break;
    }
    case LlRx:
    {
        int psize = 0;
        unsigned char *packet = (unsigned char *)malloc(MAX_PAYLOAD_SIZE);
        while (TRUE)
        {
            psize = llread(packet);
            if (psize > 0)
            {
                break;
            }
        }
        long int fileSize = getFileSizeFromPacket(packet);
        unsigned char *newFileName = getFileNameFromPacket(packet);
        printf("\n");
        printf("The file named '%s', with a filesize of %ld bytes, will be transferred.\n", newFileName, fileSize);
        printf("\n");

        FILE *newFile = fopen((char *)filename, "wb+"); // change to "newFileName" to "filename" if testing in the same computer.

        while (TRUE)
        {
            while (TRUE)
            {
                psize = llread(packet);
                if (psize > 0)
                {
                    break;
                }
            }
            if (packet[0] == 3)
            {
                break;
            }
            else if (packet[0] != 3)
            {
                unsigned char *buffer = (unsigned char *)malloc(psize);
                memcpy(buffer, packet + 4, psize - 4);
                fwrite(buffer, 1, psize - 4, newFile);
                free(buffer);
            }
            else
            {
                continue;
            }
        }
        fclose(newFile);
        llclose(fd, linklayer);
        break;
    }
    default:
        exit(EXIT_FAILURE);
        break;
    }
}