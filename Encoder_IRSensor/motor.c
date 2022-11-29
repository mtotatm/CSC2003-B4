/*DONE BY: HENG JUN HAO & Jared Teo */
/* DriverLib Includes */
#include "driverlib.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include "motor.h"

//Motor

#define target 8
#define KP 400
#define KI 195
#define KD 0.1
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

#define NOTCHLENGTH 1;
volatile int distance = 0;
volatile int timerForSpeed = 0;
volatile int timerForSpeedInOneSec = 0;


//Motor


//Motor
uint32_t timeCounter2;
volatile int notchesdetectedRight = 0;
volatile int notchesdetectedLeft = 0;
volatile int prevErrLeft = 0;
volatile int prevErrRight = 0;
volatile int sumErrLeft = 0;
volatile int sumErrRight = 0;

bool isForward = true;
bool isCompleted=false;
uint32_t testNotch = 0;
int testNotchStatus = 0;
//Motor


void initiatePortsForMotorDriver()
{
    /*Enables PWM signal for Motors */
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN2); // left motor
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0);

    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN4); // right motor
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN5);
}

void rotateForward()
{
    /*rotate both wheels forward*/
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN2); // left motor //IN3
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0); //IN4

    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN4); // right motor //IN2
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN5); //IN1
}

void rotateBackward()
{
    /*rotate both wheels backwards*/
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN2); // left motor
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN0);

    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN4); // right motor
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN5);
}
void rotateLeftF()
{
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN2); // left motor
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0);

    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN4); // right motor
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN5);
}

void rotateRightF()
{
    /*rotate Right wheel forward*/
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN2); // left motor
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0);

    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN4); // right motor
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN5);
}

void rotateLeftB()
{
    /*rotate left wheel forward*/
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN2); // left motor
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN0);

    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN4); // right motor
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN5);
}
void rotateRightB()
{
    /*rotate Right wheel forward*/
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN2); // left motor
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0);

    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN4); // right motor
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN5);
}

void staticTurnLeft(){
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN2); // left motor //IN3
     GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0); //IN4

    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN4); // right motor
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN5);

}

void stopCar()
{
    /*stop both wheels*/
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN2); // left motor
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0);

    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN4); // right motor
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN5);
}

void threePointTurn()
{
    if (timeCounter2 >= 0)
    {
        staticTurnLeft();
    }

    if (timeCounter2 >= 11)
    {
        stopCar();
        testNotch = 0;
        isCompleted=true;
    }
}
void turnLeft()
{
    if (timeCounter2 >= 0)
    {
        rotateRightF();
    }
    if (timeCounter2 >= 5)
    {
        stopCar();
        testNotch = 0;
        isCompleted=true;
    }
}

void turnRight()
{
    if (timeCounter2 >= 0)
    {
        //print(timeCounter2);
        rotateLeftF();
    }
    if (timeCounter2 >= 5)
    {
        stopCar();
        testNotch = 0;
        isCompleted=true;
    }
}


/* Timer_A PWM Configuration Parameter <right motor> for motors*/ //frequency = 1ms
Timer_A_PWMConfig pwmConfig =
    {
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_24,
        12500,
        TIMER_A_CAPTURECOMPARE_REGISTER_2,
        TIMER_A_OUTPUTMODE_RESET_SET,
        12500}; //40%
/*Create a new timer for other wheel <left motor>*/
Timer_A_PWMConfig pwmConfig1 =
    {
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_24,
        12500,
        TIMER_A_CAPTURECOMPARE_REGISTER_1,
        TIMER_A_OUTPUTMODE_RESET_SET,
        12500 //40%

};

//eUSCI_UART_Config uartConfig =
//    {
//        EUSCI_A_UART_CLOCKSOURCE_SMCLK,                // SMCLK Clock Source
//        1,                                             // BRDIV = 78
//        10,                                            // UCxBRF = 2
//        0,                                             // UCxBRS = 0
//        EUSCI_A_UART_ODD_PARITY,                       // ODD Parity
//        EUSCI_A_UART_LSB_FIRST,                        // LSB First
//        EUSCI_A_UART_ONE_STOP_BIT,                     // One stop bit
//        EUSCI_A_UART_MODE,                             // UART mode
//        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION, // Oversampling
//};

const Timer_A_UpModeConfig upConfig2 = // 0.5s second delay //Timer repeatedly counts from 0 to the value in TACCR0
    {
        TIMER_A_CLOCKSOURCE_ACLK,           // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_64,     // SMCLK/64 = 46875hz
        256,                                // 46875 tick period
        TIMER_A_TAIE_INTERRUPT_DISABLE,     // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE, // Enable CCR0 interrupt
        TIMER_A_DO_CLEAR                    // Clear counter
};
void highestSpeed()
{
    if (timeCounter2 >= 0)
    {
        pwmConfig1.dutyCycle = 12500;
        Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig1);
        pwmConfig.dutyCycle = 12500;
        Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
    }
    if (timeCounter2 >= 10)
    {
        pwmConfig1.dutyCycle = 5000;
        Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig1);
        pwmConfig.dutyCycle = 5000;
        Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
        stopCar();
        testNotch = 0;
        isCompleted=true;
    }
}

int runMotor(void)
{

    distance = 0;
    notchesdetectedRight = 0;
    notchesdetectedLeft = 0;

    timeCounter2 = 0;
    testNotch=0;
    isCompleted=false;

    /* Halting the watchdog */
//    MAP_WDT_A_holdTimer();

    initiatePortsForMotorDriver();
    rotateForward();
    /*
     * 0: Stop
     * 1: Forward
     * 2: Left
     * 3: Right
     * 4: Three Point Turn
     * 5: Highest Speed
     */
    testNotchStatus = 1;

    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION); // Set as  port as PWM output (Right Motor)
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION); // Set as  port as PWM output (Left Motor)


    /* Configuring Timer_A to have a period of approximately 80ms and an initial duty cycle of 10% of that (1000 ticks)  */
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig1);
    //    Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfig);

    /*UART*/
//    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION); // Selecting P1.2 and P1.3 in UART mode
//    UART_initModule(EUSCI_A0_BASE, &uartConfig);                                                                   // initializes UART, leaves it DISABLED
//    UART_enableModule(EUSCI_A0_BASE);                                                                              // Enable UART module

    /*HC020K PHOTOELECTRIC ENCODER (right motor)*/
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN6); // Set interrupt for HC020K P2.6
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P2, GPIO_PIN6);
    //    Interrupt_setPriority(INT_PORT2,0);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN6);

    /*HC020K PHOTOELECTRIC ENCODER (left motor)*/
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN7); // Set interrupt for HC020K P2.7
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P2, GPIO_PIN7);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN7);

    /*timer interrupt*/
    MAP_Timer_A_configureUpMode(TIMER_A1_BASE, &upConfig2); // Configuring Timer_A1 for Up Mode
                                                           //    MAP_Timer_A_configureUpMode(TIMER_A1_BASE, &upConfig2);     // Configuring Timer_A1 for Up Mode
    MAP_Interrupt_enableSleepOnIsrExit();                  // Enabling interrupts and starting the timer
                                                           //    Interrupt_setPriority(&upConfig,0);
                                                           //    Interrupt_setPriority(&upConfig2,1);
    MAP_Interrupt_enableInterrupt(INT_TA1_0);


    /* Enabling interrupts and starting the watchdog timer */
    Interrupt_enableInterrupt(INT_PORT1);
    Interrupt_enableInterrupt(INT_PORT2);
    Interrupt_enableSleepOnIsrExit();
    UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT); // enables interrupt for UART
    Interrupt_enableInterrupt(INT_EUSCIA0);                              // enables interrupt in UART
    Interrupt_enableMaster();                                            // enables master interrupt

    MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE); // Starting the Timer A1 in up mode

//    uPrintf("Going into LPM3\n\r");

//    /* Sleeping when not in use */
//    while (1)
//    {
//        PCM_gotoLPM0();
//    }
}

/* Port2 ISR*/
void PORT2_IRQHandler(void)
{

    uint32_t status;
    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P2);

    GPIO_clearInterruptFlag(GPIO_PORT_P2, status);

    if (status & GPIO_PIN6)
    {
        notchesdetectedRight++;
        distance += NOTCHLENGTH;

    }
    if (status & GPIO_PIN7)
    {
        notchesdetectedLeft++;
    }
//    if(status & GPIO_PIN6)
//    {
//        distance += NOTCHLENGTH;
//    }
}

void TA1_0_IRQHandler(void)
{
    double  pwmLeft = 0,
            pwmRight = 0;
    int     errLeft = 0,
            errRight = 0;

    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

    timeCounter2++;
    timerForSpeed++;

    if(timerForSpeed%2 == 0)
    {
        timerForSpeedInOneSec ++;
        timerForSpeed = 0;
    }

    uint32_t cmPerSec = floor(distance/timerForSpeedInOneSec);

    UART_Printf(EUSCI_A0_BASE, "Speed: %i cm/s \r\n", cmPerSec);
    UART_Printf(EUSCI_A0_BASE, "Distance: %i cm \r\n", (distance));

    if(testNotchStatus == 0)
       {
           isForward =  false;
           stopCar();
        }
        else if(testNotchStatus == 1)
          {
//            uPrintf("test1\n\r");
            isForward =  true;
            rotateForward();
          }
        else if(testNotchStatus == 2)
       {
            testNotch++;
            if(testNotch == 1){
                timeCounter2=0;
//                uPrintf("test2\n\r");
                isForward =  false;
            }
            turnLeft();
       }
        else if(testNotchStatus == 3)
        {
            testNotch++;
//            print(timeCounter2);
            if(testNotch == 1){
                timeCounter2=0;
//                uPrintf("test3\n\r");
                isForward =  false;
            }
            turnRight();
        }
        else if(testNotchStatus == 4)
        {
            testNotch++;
//            print(timeCounter2);
            if(testNotch == 1){
                timeCounter2=0;
//                uPrintf("test4\n\r");
                isForward =  false;
            }
            threePointTurn();
        }

        else if(testNotchStatus == 5)
        {
            testNotch++;
//            print(timeCounter2);
            if(testNotch == 1){
                timeCounter2=0;
//                uPrintf("test5\n\r");
                isForward =  false;
            }
            highestSpeed();
        }



    if(isForward == true){
        /*
         * 1. Doing PID for each wheel
         * 2. run at 40% DC
         * 3. target = 75% of the encoder tick per sample. 11 * 0.75 = 8.25. round to nearest whole number = 8
         *              100% = 33 notches. 33 * 0.75 = 24.75. round to nearest whole number = 25
         * 4. KP start point == 125000 / 25 = 500
         * 5. KI start point == 500 / 2 = 250
         * 6. KD start point == 0.01
         */

        if(notchesdetectedRight %20 ==0)
        {

        }

       //PID region (start)
//        errLeft = target - notchesdetectedLeft;
//        errRight = target - notchesdetectedRight;
//
//        pwmLeft += (KP * errLeft) + (KI * sumErrLeft) + (KD * prevErrLeft) ;
//        pwmRight += (KP * errRight) + (KI * sumErrRight) + (KD * prevErrRight) ;
//
//        pwmLeft = pwmConfig1.dutyCycle + pwmLeft;
//        pwmRight = pwmConfig.dutyCycle + pwmRight;
//
//        pwmConfig1.dutyCycle = MAX(MIN(12500,pwmLeft),0);
//        pwmConfig.dutyCycle = MAX(MIN(12500,pwmRight),0);
//
//
//        Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig1);
//        Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
//
//
//        sumErrLeft += errLeft;
//        sumErrRight += errRight;
//        prevErrLeft = errLeft;
//        prevErrRight = errRight;
//        notchesdetectedRight = 0;
//        notchesdetectedLeft = 0;

        //PID region (End)
    }


}

//void print(int num)
//{
//    char p[100];
//    sprintf(p, "%d", num);
//    uPrintf(p);
//    uPrintf("\n\r");
//}
//
//void printDouble(double num)
//{
//    char p[100];
//    sprintf(p, "%f", num);
//    uPrintf(p);
//    uPrintf("\n\r");
//}
//
//void uPrintf(unsigned char *TxArray)
//{
//    unsigned short i = 0;
//    while (*(TxArray + i))
//    {
//        UART_transmitData(EUSCI_A0_BASE, *(TxArray + i)); // Write the character at the location specified by pointer
//        i++;                                              // Increment pointer to point to the next character
//    }
//}
//
//void EUSCIA0_IRQHandler(void)
//{
//    unsigned char a = 0;
//    a = UART_receiveData(EUSCI_A0_BASE);
//    UART_transmitData(EUSCI_A0_BASE, a);
//}


