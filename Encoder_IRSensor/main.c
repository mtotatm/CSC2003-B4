/*
 * main.c
 *
 *  Created on: 27 Oct 2022
 *      Author: Calvert
 */

/* DriverLib Includes */
#include "driverlib.h"

//Libraries to run
#include "adc_scanner.h"
#include "uart_print.h"
#include "motor.h"
#include <stdio.h>
//#include <string.h>
//#include <math.h>

/* Main to run program (CNTRL + Space for intellisense)*/
int main(void)
{
    /* Halting the Watchdog  */
    MAP_WDT_A_holdTimer();

    initPrint();
    //uPrintf("HELLO START BARCODE\n\r");
//    UART_Printf(EUSCI_A2_BASE, "HELLO START BARCODE \r\n");
    runMotor();
    adcScannerInit();
//    uPrintInt(200);

    /* Enter sleep mode*/
    while (1)
    {
//        PCM_gotoLPM3();
        MAP_PCM_gotoLPM0();
    }
//    return 0;
}
