// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <stdio.h>
#include <string.h>


void applicationLayer(const char *serialPort, const char *role, int baudRate, int nTries, int timeout, const char *filename)
{
    LinkLayer connectionParameters;
    strcpy(connectionParameters.serialPort, serialPort);
    connectionParameters.baudRate = baudRate;
    connectionParameters.nRetransmissions = nTries;
    connectionParameters.timeout = timeout;
    
    if (strcmp(role, "tx") == 0)
    {
        connectionParameters.role = LlTx;
    }
    else if (strcmp(role, "rx") == 0)
    {
        connectionParameters.role = LlRx;
    }
    else
    {
        printf("Invalid role: %s\n", role);
        return;
    }

    if (llopen(connectionParameters) < 0)
    {
        printf("Error: failed to open the link layer connection.\n");
        return;
    }

    if (connectionParameters.role == LlTx)  
    {
        FILE *file = fopen(filename, "rb");
        if (file == NULL)
        {
            printf("Error: could not open file %s for reading.\n", filename);
            llclose(FALSE);
            return;
        }

        unsigned char buffer[MAX_PAYLOAD_SIZE];
        size_t bytesRead;
        while ((bytesRead = fread(buffer, 1, MAX_PAYLOAD_SIZE, file)) > 0)
        {
            if (llwrite(buffer, bytesRead) < 0)
            {
                printf("Error: failed to send data to the link layer.\n");
                fclose(file);
                llclose(FALSE);
                return;
            }
        }

        fclose(file);

        if (llclose(TRUE) < 0)
        {
            printf("Error: failed to close the link layer connection.\n");
            return;
        }

        printf("File transmission completed successfully.\n");
    }
    else if (connectionParameters.role == LlRx)  
    {
        FILE *file = fopen(filename, "wb");
        if (file == NULL)
        {
            printf("Error: could not open file %s for writing.\n", filename);
            llclose(FALSE);
            return;
        }

        unsigned char buffer[MAX_PAYLOAD_SIZE];
        int bytesRead;
        while ((bytesRead = llread(buffer)) > 0)
        {
            if (fwrite(buffer, 1, bytesRead, file) < bytesRead)
            {
                printf("Error: failed to write data to the file.\n");
                fclose(file);
                llclose(FALSE);
                return;
            }
        }

        fclose(file);

        if (llclose(TRUE) < 0)
        {
            printf("Error: failed to close the link layer connection.\n");
            return;
        }

        printf("File reception completed successfully.\n");
    }
    else
    {
        printf("Error: invalid role specified.\n");
    }
}