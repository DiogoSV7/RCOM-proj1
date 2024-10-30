// Application layer protocol header.
// NOTE: This file must not be changed.

#ifndef _APPLICATION_LAYER_H_
#define _APPLICATION_LAYER_H_

#include <stdio.h>


#define MAX_READ_ATTEMPTS 10
// Application layer main function.
// Arguments:
//   serialPort: Serial port name (e.g., /dev/ttyS0).
//   role: Application role {"tx", "rx"}.
//   baudrate: Baudrate of the serial port.
//   nTries: Maximum number of frame retries.
//   timeout: Frame timeout.
//   filename: Name of the file to send / receive.
void applicationLayer(const char *serialPort, const char *role, int baudRate, int nTries, int timeout, const char *filename);

long int getFileSizeFromPacket(unsigned char* packet);

unsigned char* getFileNameFromPacket(unsigned char* packet);

long int getFileSize(FILE *file);

unsigned char *createControlPacket(const char *filename, long int filesize, unsigned int *length);

void createDataPacket(FILE* file, unsigned char *dataPacket, int dataSize, unsigned char identifier);

#endif // _APPLICATION_LAYER_H_
