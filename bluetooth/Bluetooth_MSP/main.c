/* DriverLib Includes */
#include <driverlib.h>
#include <UART_Driver.h>

/*Input Buffer*/
#define BUFFER_SIZE 128
char Buffer[BUFFER_SIZE];

/* Standard Includes */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>

#include <math.h>
#include <string.h>
int timer = 0;



//![Simple UART Config]
/* UART Configuration Parameter. These are the configuration parameters to
 * make the eUSCI A UART module to operate with a 9600 baud rate. These
 * values were calculated using the online calculator that TI provides at:
 * http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
 */

const eUSCI_UART_Config UART0Config =
{
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,                 // SMCLK Clock Source
        19,                                              // BRDIV = 1
        8,                                             // UCxBRF = 10
        85,                                              // UCxBRS = 0
        EUSCI_A_UART_NO_PARITY,                        // ODD Parity
        EUSCI_A_UART_LSB_FIRST,                         // LSB First
        EUSCI_A_UART_ONE_STOP_BIT,                      // One stop bit
        EUSCI_A_UART_MODE,                              // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION,  // Oversampling
};


//Timer_A UpMode Configuration Parameter
const Timer_A_UpModeConfig upConfig =
{
        TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source = 3Mhz
        TIMER_A_CLOCKSOURCE_DIVIDER_64,         // TACLK = 3MHz / 64
        46875,                           // 46875 ticks (CCR0)
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,    // Enable CCR0 interrupt
        TIMER_A_DO_CLEAR                        // Clear value
};

/*UART2Config for bluetooth HC-05 module*/
const eUSCI_UART_Config UART2Config =
{
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,                 // SMCLK Clock Source
        19,                                              // BRDIV = 1
        8,                                             // UCxBRF = 10
        85,                                              // UCxBRS = 0
        EUSCI_A_UART_NO_PARITY,                        // ODD Parity
        EUSCI_A_UART_LSB_FIRST,                         // LSB First
        EUSCI_A_UART_ONE_STOP_BIT,                      // One stop bit
        EUSCI_A_UART_MODE,                              // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION,  // Oversampling
};


int main(void)
{

    /* Halting WDT  */
    WDT_A_holdTimer();

    /*Ensure MSP432 is Running at 3 MHz*/
    FlashCtl_setWaitState(FLASH_BANK0, 2);
    FlashCtl_setWaitState(FLASH_BANK1, 2);
    PCM_setCoreVoltageLevel(PCM_VCORE1);
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_3);

    /*Set Port 1 and 2 as output for testing purpose*/
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN1);
    //MAP_GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN1);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);


    /*Config S1 and 2 button*/
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1); //set S1 button as input with pull up resistor
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN4); //set S2 button as input with pull up resistor

    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN4);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN4);

    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN1);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1);

    /*enable port1 interrupt*/
    MAP_Interrupt_enableInterrupt(INT_PORT1);

    /*Initialize required hardware peripherals for the MSP432 serial*/
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_UART_initModule(EUSCI_A0_BASE, &UART0Config);
    MAP_UART_enableModule(EUSCI_A0_BASE);
    MAP_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA0);


    /Initialize required hardware peripherals for HC-05*/
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_UART_initModule(EUSCI_A2_BASE, &UART2Config);
    MAP_UART_enableModule(EUSCI_A2_BASE);
    MAP_UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA2);


    /*Configuring Timer_A1 for Up Mode*/
    MAP_Timer_A_configureUpMode(TIMER_A1_BASE, &upConfig);

    /*Enabling interrupts and starting the timer*/
    MAP_Interrupt_enableSleepOnIsrExit();
    MAP_Interrupt_enableInterrupt(INT_TA1_0);

    MAP_Interrupt_enableMaster();



    while(1){
        MAP_PCM_gotoLPM0();
    }


}



void PORT1_IRQHandler(void)
{
    uint32_t status = GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
    int counter = 0;

    /*S2 Button to toggle timer*/
    if (status & GPIO_PIN4)
    {
        if (start == 0 ){
            MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);

            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);

            start++;
        }
        else if (start == 1){
            //UART_Printf(EUSCI_A2_BASE, "Stop Data\n\r");
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);
            start--;
            MAP_Timer_A_stopTimer(TIMER_A1_BASE);

        }

    }

    /*S1 Button to togle input mode*/
    if (status & GPIO_PIN1)
    {
        if (start == 0 )
        {
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
            UART_Gets(EUSCI_A2_BASE,Buffer,BUFFER_SIZE);
            UART_Printf(EUSCI_A0_BASE,"%s",Buffer);

            start++;
        }
        else if (start == 1){
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
            start--;

        }

    }


    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, status);

}

void TA1_0_IRQHandler(void)
{
    /*convert double to string*/
    //char p[100];
    //sprintf(p,"%f",data);


    //send data
    timer++;

    UART_Printf(EUSCI_A2_BASE,"Message-Hi\r\n");
    UART_Printf(EUSCI_A2_BASE,"Mapping-Node 1: x=1 y=2\r\n");
    UART_Printf(EUSCI_A2_BASE,"IRSensor-A\r\n");



    timer = 0;

    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);
}


