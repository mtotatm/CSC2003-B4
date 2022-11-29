/*
 * -------------------------------------------
 *    MSP432 DriverLib - v3_21_00_05 
 * -------------------------------------------
 *
 * --COPYRIGHT--,BSD,BSD
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
/*******************************************************************************
 * MSP432 ADC14 - Single Channel Sample Repeat
 *
 * Description: This code example will demonstrate the basic functionality of
 * of the DriverLib ADC APIs with sampling a single channel repeatedly. Each
 * time the ADC conversion occurs, the result is stored into a local variable.
 * The sample timer is used to continuously grab a new value from the ADC
 * module using a manual iteration that is performed in the ADC ISR. A
 * normalized ADC value with respect to the 3.3v Avcc is also calculated using
 * the FPU.
 *
 *                MSP432P401
 *             ------------------
 *         /|\|                  |
 *          | |                  |
 *          --|RST         P5.5  |<--- A0 (Analog Input)
 *            |                  |
 *            |                  |
 *            |                  |
 *            |                  |
 *            |                  |
 *
 * Author: Timothy Logan
 ******************************************************************************/
/* DriverLib Includes */
#include "driverlib.h"

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* This file's Header file*/
#include "adc_scanner.h"
#include "uart_print.h"

/* Define*/
#define CLEARCOND 3000
#define BARCODELEN 29

/* Statics */
static volatile uint16_t curADCResult;
static volatile float normalizedADCRes;

// Global var
volatile bool barCodeActivated = false;
volatile int timer = 0;
volatile int timerArr[BARCODELEN] = {0};
volatile char actualBinaryContainer[15]= {0};
volatile int count = 0;

//Global var to print to excel
//volatile bool scan = false;
//volatile int countforGRAPH = 0;
volatile int timeCount = 0;
//volatile int scanned_cache[5000];
//volatile int time_cache[5000];

/* Timer_A UpMode Configuration Parameter */
const Timer_A_UpModeConfig upConfig =
{
       TIMER_A_CLOCKSOURCE_SMCLK,               // SMCLK Clock Source = 3Mhz
       TIMER_A_CLOCKSOURCE_DIVIDER_3,           // TACLK = 3MHz / 64
       1000,                                    // 46875 ticks (CCR0)
       TIMER_A_TAIE_INTERRUPT_DISABLE,          // Disable Timer interrupt
       TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,     // Enable CCR0 interrupt
       TIMER_A_DO_CLEAR                         // Clear value
};

/* Init function for scanner*/
int adcScannerInit(void)
{
    barCodeActivated = false;
    /*timer interrupt*/
    MAP_Timer_A_configureUpMode(TIMER_A3_BASE, &upConfig);      // Configuring Timer_A1 for Up Mode
//    MAP_Interrupt_enableInterrupt(INT_TA1_0);
    MAP_Timer_A_startCounter(TIMER_A3_BASE, TIMER_A_UP_MODE);

    /* Enable interrupt */
    Interrupt_enableInterrupt(INT_TA3_0);
    /* Enable interrupts globally */
//    Interrupt_enableMaster();

    /* Setup Light for LED_1 (RED)*/
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);

    /* Initializing Variables */
    curADCResult = 0;

    /* Setting Flash wait state */
    MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
    MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);
    
    /* Setting DCO to 48MHz  */
    MAP_PCM_setPowerState(PCM_AM_LDO_VCORE1);
//    MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_48);

    /* Enabling the FPU for floating point operation */
    MAP_FPU_enableModule();
    MAP_FPU_enableLazyStacking();

    /* Initializing ADC (MCLK/1/4) */
    MAP_ADC14_enableModule();
    MAP_ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_4, 0);
            
    /* Configuring GPIOs (5.5 A0) */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN5, GPIO_TERTIARY_MODULE_FUNCTION);

    /* Configuring ADC Memory */
    MAP_ADC14_configureSingleSampleMode(ADC_MEM0, true);
    MAP_ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A0, false);

    /* Configuring Sample Timer */
    MAP_ADC14_enableSampleTimer(ADC_MANUAL_ITERATION);

    /* Enabling/Toggling Conversion */
    MAP_ADC14_enableConversion();
    MAP_ADC14_toggleConversionTrigger();

    /* Enabling interrupts */
    MAP_ADC14_enableInterrupt(ADC_INT0);
    MAP_Interrupt_enableInterrupt(INT_ADC14);
    MAP_Interrupt_enableMaster();

    return 0;
}

/* ADC Interrupt Handler. This handler is called whenever there is a conversion
 * that is finished for ADC_MEM0.
 */
void ADC14_IRQHandler(void)
{
    static int light = 1; // 1 is white, 2 is black

    /* Get interrupt status N clear flag*/
    uint64_t status = MAP_ADC14_getEnabledInterruptStatus();
    MAP_ADC14_clearInterruptFlag(status);

    /* Detect if ADC and if status is true*/
    if (ADC_INT0 & status)
    {
        /* Get adc results from adc memory 0*/
        curADCResult = MAP_ADC14_getResult(ADC_MEM0);

        //(currentResult * voltage) / (2^14)+1
        normalizedADCRes = (curADCResult * 3.3) / 16384;

    //    if(barCodeActivated && scan && countforGRAPH < 5000)
    //    {
    //        scanned_cache[countforGRAPH] = (int)curADCResult;
    //        time_cache[countforGRAPH] = timeCount;
    //        countforGRAPH++;
    //    }

        /* Threshold detection for black and white (Lower number = white | higher number = black)*/
        if(curADCResult <= 750 && light == 1 && timer > 100)
        {
            if(barCodeActivated == true)
            {
                TriggerIRTimerInput();
                UART_Printf(EUSCI_A0_BASE, "%i \r\n", curADCResult);
            }
            GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
            light = 2;
        }
        else if(curADCResult >= 800 && light == 2 && timer > 100)
        {
            if(barCodeActivated == false)
            {
                barCodeActivated = true;
                timer=0;
            }
            if(barCodeActivated == true)
            {
                TriggerIRTimerInput();
                UART_Printf(EUSCI_A0_BASE, "%i \r\n", curADCResult);
            }
            GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
            light = 1;
        }

        //Unsure
        MAP_ADC14_toggleConversionTrigger();
    }
}

// Timer Trigger IR INPUT DATA
void TriggerIRTimerInput(void)
{


    if(timer <= 2)
        return;

    UART_Printf(EUSCI_A0_BASE, "%i \r\n", count);

    if(count == BARCODELEN - 1)
    {
        timerArr[count] += timer;
        GetMyChar();
        TimeArrayClear();
        barCodeActivated = false;
        count = 0;
        timer = 0;
        return;
    }
    else
    {
        timerArr[count] += timer;
    }

    timer = 0;
    count++;
}

// convert the timerArr into binary format using the data array for IR sensor
void GetMyChar(void)
{
    int i = 0;
    int minTimer = 0;
    int maxTimer = 0;

    for(i = 0; i < BARCODELEN; i++)
    {
        if(i == 0)
        {
            minTimer = timerArr[i];
            maxTimer = timerArr[i];
        }
        else if(minTimer > timerArr[i])
        {
            minTimer = timerArr[i];
        }
        else if(maxTimer < timerArr[i])
        {
            maxTimer = timerArr[i];
        }
    }


    for(i = 10; i < 19; i++)
    {
        int minDifference = 0;
        int maxDifference = 0;

        minDifference = timerArr[i] - minTimer;
        maxDifference = maxTimer - timerArr[i];

        if(minDifference > maxDifference) // small bar code
        {
            if(i%2 == 0)
            {
                strncat(actualBinaryContainer, "1", 1);
            }
            else
            {
                strncat(actualBinaryContainer, "0", 1);
            }
        }
        else // big bar code
        {
            if(i%2 == 0)
            {
                strncat(actualBinaryContainer, "111", 3);
            }
            else
            {
                strncat(actualBinaryContainer, "000", 3);
            }
        }
    }

//    UART_Printf(EUSCI_A0_BASE, "IR- FINALE================================================ \r\n");

    if(actualBinaryContainer != "")
    {
        char helper = findCharacter(actualBinaryContainer);
        UART_Printf(EUSCI_A0_BASE, "IR Binary- %s \r\n",actualBinaryContainer);
        UART_Printf(EUSCI_A0_BASE, "IR- %c \r\n",helper);
        TimeArrayClear();


        // if(scan)
        // {
        //     UART_Printf(EUSCI_A0_BASE, "IR- SCANNER================================================ \r\n");

        //     for(i = 0; i < 5000; i++)
        //     {
        //         UART_Printf(EUSCI_A0_BASE, "BINARY: %i, %i \r\n",scanned_cache[i], time_cache[i]);
        //     }

        //     UART_Printf(EUSCI_A0_BASE, "IR- SCANNER================================================ \r\n");
        // }
        // scan = false;

    }

}

//Memset Array to 0
void TimeArrayClear()
{
    memset(timerArr, 0, BARCODELEN);
    int t = 0;
    for(t = 0; t < sizeof(actualBinaryContainer); ++t)
    {
        actualBinaryContainer[t] = NULL;
    }
}

// Timer
void TA3_0_IRQHandler(void)
{
    //Clear Interrupt
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);
    timer++;
    timeCount++;

    // timer is 1ms, 1000ms is 1 second
    if(barCodeActivated && timer >= CLEARCOND)
    {
       //Clear IR timer
        barCodeActivated = false;
        TimeArrayClear();
//        MAP_ADC14_disableConversion();
//        MAP_ADC14_clearInterruptFlag();
        count = 0;
        UART_Printf(EUSCI_A0_BASE, "CLEAR \r\n");
    }
}
