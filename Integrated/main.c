/*
 * main.c
 *
 *  Created on: 27 Oct 2022
 *      Author: Calvert
 */

/* DriverLib Includes */
#include "driverlib.h"

//Libraries to run
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "map_dijkstra.h"
//#include "motor.h"
#include "adc_scanner.h"
#include "uart_print.h"
/* Main to run program (CNTRL + Space for intellisense)*/
int main(void)
{
    /* Halting the Watchdog  */
    MAP_WDT_A_holdTimer();

    initPrint();
    initialise_Sensor();
    adcScannerInit();
    initmotor();
    //adcScannerInit();
    //uPrintf("HELLO START BARCODE\n\r");
    //MSP UART
    //UART_Printf(EUSCI_A2_BASE, "Message-HELLO START BARCODE \r\n");

    //Bluetooth UART
//    uPrintInt(200);
    while(1){
        MAP_PCM_gotoLPM0();
    }
    return 0;
}
