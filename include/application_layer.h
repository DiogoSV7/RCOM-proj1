// Application layer protocol header.
// NOTE: This file must not be changed.

#ifndef _APPLICATION_LAYER_H_
#define _APPLICATION_LAYER_H_

#include <stdio.h>

// Application layer main function.
// Arguments:
//   serialPort: Serial port name (e.g., /dev/ttyS0).
//   role: Application role {"tx", "rx"}.
//   baudrate: Baudrate of the serial port.
//   nTries: Maximum number of frame retries.
//   timeout: Frame timeout.
//   filename: Name of the file to send / receive.
void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename);

long int findFileSize(FILE *file);

unsigned char *buildControlPacket(const char *filename, long int filesize, unsigned int *length);

void buildDataPacket(FILE* file, unsigned char *dataPacket, int dataSize, unsigned char identifier);

long int extractFileSize(unsigned char* packet);

unsigned char* extractFileName(unsigned char* packet);

void extractData(unsigned char* packet, unsigned char* buffer, int datasize);

#endif // _APPLICATION_LAYER_H_
