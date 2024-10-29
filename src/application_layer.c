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

long int findFileSize(FILE *file){
    fseek(file, 0L, SEEK_END);
    long int filesize = ftell(file);
    fseek(file, 0, SEEK_SET);

    return filesize;
}

unsigned char *buildControlPacket(const char *filename, long int filesize, unsigned int *length){
    int L1 = 0;
    int L2 = strlen(filename);
    int packetpos = 0;
    long int auxfilesize = filesize;

    while(auxfilesize > 0){
        L1++;
        auxfilesize /= 255;
    }

    *length = 3 + L1 + 2 + L2; // (C, T1,L1, V, T2,L2, V2)

    unsigned char *packet = (unsigned char*)malloc(*length);

    packet[packetpos] = 1;
    packetpos++;
    packet[packetpos] = 0;
    packetpos++;
    packet[packetpos] = L1;
    
    for(int i = L1 + packetpos; i > packetpos; i--){
        packet[i] = (0XFF & filesize);
        filesize >>= 8;
    }
    packetpos += L1 + 1;
    packet[packetpos] = 1;
    packetpos++;
    packet[packetpos] = L2;
    packetpos++;

    for(int j = 0; j < L2; j++){
        packet[packetpos + j] = filename[j];
    }

    return packet;
}

void buildDataPacket(FILE* file, unsigned char *dataPacket, int dataSize, unsigned char identifier){

    dataPacket[0] = 2;
    dataPacket[1] = identifier;
    dataPacket[2] = (dataSize >> 8) & 0xFF;
    dataPacket[3] = dataSize & 0xFF;

    fread(dataPacket + 4, 1, dataSize, file);
}

long int extractFileSize(unsigned char* packet){
    unsigned char numBytes = packet[2];
    unsigned char aux_buf[numBytes];
    long int rxFileSize = 0;
    memcpy(aux_buf, packet + 3, numBytes);
    for(int i = 0; i < numBytes; i++) {
        rxFileSize |= (aux_buf[numBytes-1-i] << (8*i)); // extract LSB bytes from original packet and put them from right to left as expected
    }
    return rxFileSize;
}

unsigned char* extractFileName(unsigned char* packet){
    unsigned char numBytes = packet[2]; // file
    unsigned char filenameBytes = packet[3+numBytes+1];
    unsigned char *filename = (unsigned char*) malloc(filenameBytes);
    memcpy(filename, packet+3+numBytes+2, filenameBytes);
    return filename;
}

void extractData(unsigned char* packet, unsigned char* buffer, int datasize){
    memcpy(buffer, packet + 4, datasize);
}

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer linklayer;
    strcpy(linklayer.serialPort, serialPort);
    linklayer.role = (strcmp(role, "rx") == 0) ? LlRx : LlTx;
    linklayer.baudRate = baudRate;
    linklayer.nRetransmissions = nTries;
    linklayer.timeout = timeout;

    int fd = llopen(linklayer);
    if(fd < 0){
        perror("Connection between Tx and Rx failed\n");
        exit(-1);
    }
    else{
        printf("Connection between Tx and Rx succed\n");
    }

    switch (linklayer.role){

        case LlTx:{

            FILE* file = fopen(filename, "rb");
            if(file == NULL){
                perror("Error opening file");
                exit(-1);
            }
            
            long int filesize = findFileSize(file);
            unsigned int cplength;
            unsigned char* controlPacket = buildControlPacket(filename, filesize, &cplength);
            

            if(llwrite(controlPacket, cplength) == -1){
                perror("Error while writing start control packet\n");
                exit(-1);
            }
            else{
                printf("Sucess while writing start control packet\n");
            }

            long int bytes = filesize;
            unsigned char identifier = 0;

            while(bytes > 0){
                printf("Value of bytes: %ld\n", bytes);
                int dataSize = bytes > (long int) (MAX_PAYLOAD_SIZE - 4) ? (MAX_PAYLOAD_SIZE - 4) : bytes;
                int dataPacketSize = dataSize + 4;
                unsigned char* dataPacket = (unsigned char*) malloc(dataPacketSize);
                buildDataPacket(file, dataPacket, dataSize, identifier);

                if(llwrite(dataPacket, dataPacketSize) == -1){
                    perror("Error while writing data packet\n");
                    exit(-1);
                }
                bytes -= dataSize;
                identifier = (identifier + 1) % 255;
            }

            controlPacket[0] = 3;
            if(llwrite(controlPacket, cplength) == -1){
                perror("Error while writing end control packet\n");
                exit(-1);
            }
            else{
                printf("Sucess while writing end control packet\n");
            }
            llclose(fd, linklayer);
            break;
        }

        case LlRx:{
            unsigned char *packet = (unsigned char*) malloc(MAX_PAYLOAD_SIZE);
            int packetsize = 0;
            while(1){
                packetsize = llread(packet);
                if(packetsize > 0){
                    break;
                }
            }
            // read control packet and now need to extract filename aswell as filesize
            long int rxFileSize = extractFileSize(packet);
            unsigned char* rxFileName = extractFileName(packet);

            FILE* rxFile = fopen((char *) filename, "wb+"); // update rxFileName to filename if testing in the same computer with many terminals.

            while(1){
                while(1){
                    packetsize = llread(packet);
                    if(packetsize > 0){
                        break;
                    }
                }
                if(packet[0] == 3) break;
                else if(packet[0] != 3){
                    unsigned char *buffer = (unsigned char*)malloc(packetsize);
                    extractData(packet, buffer, packetsize - 4);
                    fwrite(buffer, 1, packetsize -4, rxFile);
                    free(buffer);
                }
                else continue;
            }
            fclose(rxFile);
            llclose(fd, linklayer);
            break;
        }
        default:
            exit(-1);
            break;
    }
}