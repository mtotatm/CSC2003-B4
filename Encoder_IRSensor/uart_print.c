/*
 * uart_print.c
 *
 *  Created on: 2 Nov 2022
 *      Author: Calvert
 */
/* DriverLib includes*/
#include "driverlib.h"

/* Standard Lib*/
#include <stdio.h>
#include <stdint.h>
#include <string.h>

/* Print Header File*/
#include "uart_print.h"

const eUSCI_UART_Config UART0Config =
{
    EUSCI_A_UART_CLOCKSOURCE_SMCLK,                 // SMCLK Clock Source
    19,                                             // BRDIV = 78
    8,                                             // UCxBRF = 2
    85,                                              // UCxBRS = 0
    EUSCI_A_UART_NO_PARITY,                        // ODD Parity
    EUSCI_A_UART_LSB_FIRST,                         // LSB First
    EUSCI_A_UART_ONE_STOP_BIT,                      // One stop bit
    EUSCI_A_UART_MODE,                              // UART mode
    EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION,  // Oversampling
};
const eUSCI_UART_Config UART2Config =
{
    EUSCI_A_UART_CLOCKSOURCE_SMCLK,                 // SMCLK Clock Source
    19,                                             // BRDIV = 78
    8,                                             // UCxBRF = 2
    85,                                              // UCxBRS = 0
    EUSCI_A_UART_NO_PARITY,                        // ODD Parity
    EUSCI_A_UART_LSB_FIRST,                         // LSB First
    EUSCI_A_UART_ONE_STOP_BIT,                      // One stop bit
    EUSCI_A_UART_MODE,                              // UART mode
    EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION,  // Oversampling
};



/* Do not run this as they are called in function for print*/
void initPrint(void)
{
    static bool firstInit = true;
    if(!firstInit)
        return;

    //Initialize required hardware peripherals for HC-05
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_UART_initModule(EUSCI_A0_BASE, &UART0Config);
    MAP_UART_enableModule(EUSCI_A0_BASE);
    MAP_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA0);


    //Initialize required hardware peripherals for HC-05
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_UART_initModule(EUSCI_A2_BASE, &UART2Config);
    MAP_UART_enableModule(EUSCI_A2_BASE);
    MAP_UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA2);



    /* Enable interrupt */
    Interrupt_enableInterrupt(INT_TA0_0);
    MAP_Interrupt_enableMaster();
    UART_Printf(EUSCI_A2_BASE, "Init Print Module \r\n");

    firstInit = false;
}

void UART_Write(uint32_t UART, uint8_t *Data, uint32_t Size)
{
    uint32_t i;
    for(i = 0; i < Size; i++)
    {
        MAP_UART_transmitData(UART, Data[i]);
    }
}

uint32_t UART_Read(uint32_t UART, uint8_t *Data, uint32_t Size)
{
    uint32_t i;
    int8_t c;

    switch(UART)
    {
    case EUSCI_A0_BASE:
        for(i = 0; i < Size; i++)
        {
            bool checkEmpty = true;
            if (UARTA0ReadIndex == UARTA0WriteIndex)
                checkEmpty = true;
            else
                checkEmpty = false;

            if(checkEmpty)
            {
                return i;
            }
            else
            {
                c = UARTA0Data[UARTA0ReadIndex];

                MAP_Interrupt_disableMaster();
                UARTA0ReadIndex = (UARTA0ReadIndex + 1) % UARTA0_BUFFERSIZE;
                MAP_Interrupt_enableMaster();

                Data[i] = c;
            }
        }
    break;

    case EUSCI_A2_BASE:
        for(i = 0; i < Size; i++)
        {
            bool checkEmpty = true;
            if (UARTA2ReadIndex == UARTA2WriteIndex)
                checkEmpty = true;
            else
                checkEmpty = false;

            if(checkEmpty)
            {
                return i;
            }
            else
            {
                c = UARTA2Data[UARTA2ReadIndex];

                MAP_Interrupt_disableMaster();
                UARTA2ReadIndex = (UARTA2ReadIndex + 1) % UARTA2_BUFFERSIZE;
                MAP_Interrupt_enableMaster();

                Data[i] = c;
            }
        }
    break;
    /*More UART reading modules go here*/
    default:
        return 0;
    }

    return i;
}

/*A basic gets for the MSP432. In order to use it properly you need to initialize the correct UART peripheral.
 * USAGE...
 * UART_Gets(EUSCI_A0_BASE, InputBuffer, SizeOfInputBuffer)*/

int UART_Gets(uint32_t UART, char *b, int size)
{
    char c;
    uint32_t i = 0;

    while(1)
    {
        if(UART_Read(UART, (uint8_t*)&c, 1) != 0)
        {
           /*put a '\n' and '\r' if it fits on the buffer*/
           if(c == '\n' || c == '\r')
           {
               if(i + 3 > size)
               {
                   return size + 1;
               }

               b[i++] = '\r';
               b[i++] = '\n';
               b[i++] = 0;

               return i;
           }
           /*erase data from buffer if backspace is received*/
           else if(c == 127 || c == 8)
           {
               i--;
               b[i] = 0;
           }
           /*store character on the buffer*/
           else
           {
               if(i < size)
               {
                   b[i++] = c;
               }
               else
               {
                   return size + 1;
               }
           }
        }
    }
}

/*A basic printf for the MSP432. In order to use it properly you need to initialize the correct UART peripheral.
 * The following formats are supported:
 * %c = for char variables
 * %s = for string variables
 * %i = for unsigned integers
 * USAGE...
 * MSPrintf(EUSCI_A0_BASE, "Formated string %c, %s, %i", character, string, integer)*/

void UART_Printf(uint32_t UART, const char *fs, ...)
{
    va_list valist;
    va_start(valist, fs);

    int i;
    char *s;
    int tmpVal;

    while(*fs)
    {
        if(*fs != '%')
        {
            UART_Write(UART, (uint8_t*)fs, 1);
            fs++;
        }
        else
        {
            switch(*++fs)
            {
            case 'c':
                i = va_arg(valist, int);
                UART_Write(UART, (uint8_t*)&i, 1);
                break;
            case 's':
                s = va_arg(valist, char*);
                while(*s)
                {
                    UART_Write(UART, (uint8_t*)s, 1);
                    s++;
                }
                break;
            case 'i':
                i = va_arg(valist, int);
                if(i == 0)
                {
                    tmpVal = '0';
                    UART_Write(UART, (uint8_t*)&tmpVal, 1);
                }

                if(i < 0)
                {
                    tmpVal = '-';
                    UART_Write(UART, (uint8_t*)&tmpVal, 1);
                }

                char b[10];
                int digit = i;

                uint8_t i = 0;
                while(digit)
                {
                    b[i++] = digit % 10;
                    digit /= 10;
                }

                while(i)
                {
                    tmpVal = '0' + b[i-1];
                    UART_Write(UART, (uint8_t*)&tmpVal, 1);
                    i--;
                }
                break;
            }

            ++fs;
        }
    }
}


void EUSCIA0_IRQHandler(void)
{
    uint8_t c;
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A0_BASE);

    MAP_UART_clearInterruptFlag(EUSCI_A0_BASE, status);

    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        c = MAP_UART_receiveData(EUSCI_A0_BASE);

        bool chkFull = true;
        if ((UARTA0WriteIndex + 1) % UARTA0_BUFFERSIZE == UARTA0ReadIndex)
            chkFull = true;
        else
            chkFull = false;

        if(chkFull)
        {
            //TODO: Buffer Overflow, add handler here
        }
        else
        {
            UARTA0Data[UARTA0WriteIndex] = c;
            UARTA0WriteIndex = (UARTA0WriteIndex + 1) % UARTA0_BUFFERSIZE;

            //Transmit data only if it made it to the buffer
            MAP_UART_transmitData(EUSCI_A0_BASE, c);
        }
    }
}



void EUSCIA2_IRQHandler(void)
{
    uint8_t c;
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A2_BASE);
    MAP_UART_clearInterruptFlag(EUSCI_A2_BASE, status);

    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT)
    {
        c = MAP_UART_receiveData(EUSCI_A2_BASE);

        bool chkFull = true;
        if ((UARTA2WriteIndex + 1) % UARTA2_BUFFERSIZE == UARTA2ReadIndex)
            chkFull = true;
        else
            chkFull = false;

        if (chkFull)
        {
            /*TODO: Buffer Overflow, add handler here*/
        }
        else
        {
            UARTA2Data[UARTA2WriteIndex] = c;
            UARTA2WriteIndex = (UARTA2WriteIndex + 1) % UARTA2_BUFFERSIZE;
        }
    }
}



