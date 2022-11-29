#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "driverlib.h"

#define target 8
#define KP 400
#define KI 195
#define KD 0.1
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

void initiatePortsForMotorDriver();
void rotateForward();
void rotateBackward();
void rotateLeftF();
void rotateRightF();
void rotateLeftB();
void rotateRightB();
void staticTurnLeft();
void stopCar();
void threePointTurn();
void turnLeft();
void turnRight();

//Motor
uint32_t turnExecutionTime;
uint32_t timeCounter1;
volatile int notchesdetectedRight = 0;
volatile int notchesdetectedLeft = 0;
volatile int prevErrLeft = 0;
volatile int prevErrRight = 0;
volatile int sumErrLeft = 0;
volatile int sumErrRight = 0;

bool isForward = false;
bool isCompleted=false;
uint32_t testNotch = 0;
int testNotchStatus = 0;


/* Timer_A PWM Configuration Parameter <right motor> for motors*/ //frequency = 1ms
Timer_A_PWMConfig pwmConfig =
    {
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_24,
        12500,
        TIMER_A_CAPTURECOMPARE_REGISTER_2,
        TIMER_A_OUTPUTMODE_RESET_SET,
        5000 //40%
    };
/*Create a new timer for other wheel <left motor>*/
Timer_A_PWMConfig pwmConfig1 =
    {
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_24,
        12500,
        TIMER_A_CAPTURECOMPARE_REGISTER_1,
        TIMER_A_OUTPUTMODE_RESET_SET,
        5000 //40%

};

const Timer_A_UpModeConfig upConfigMotor = // 0.5s second delay //Timer repeatedly counts from 0 to the value in TACCR0
    {
        TIMER_A_CLOCKSOURCE_ACLK,           // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_64,     // SMCLK/64 = 46875hz
        256,                                // 46875 tick period
        TIMER_A_TAIE_INTERRUPT_DISABLE,     // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE, // Enable CCR0 interrupt
        TIMER_A_DO_CLEAR                    // Clear counter
};

eUSCI_UART_Config uartConfig =
    {
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,                // SMCLK Clock Source
        1,                                             // BRDIV = 78
        10,                                            // UCxBRF = 2
        0,                                             // UCxBRS = 0
        EUSCI_A_UART_ODD_PARITY,                       // ODD Parity
        EUSCI_A_UART_LSB_FIRST,                        // LSB First
        EUSCI_A_UART_ONE_STOP_BIT,                     // One stop bit
        EUSCI_A_UART_MODE,                             // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION, // Oversampling
};


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
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0);                //IN4

    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN4); // right motor //IN2
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN5);                //IN1

    isForward = true;
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
    /*rotate left wheel only*/

    pwmConfig1.dutyCycle = 11500; //92% dc adjustment for accurate turn
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig1);

    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN2); // left motor
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0);

    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN4); // right motor
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN5);

    isForward = false;
}


void rotateRightF()
{

    /*rotate Right wheel forward*/
    pwmConfig.dutyCycle = 10875; //87% dc adjustment for accurate turn
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);

    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN2); // left motor
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0);

    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN4); // right motor
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN5);

    isForward = false;
}

void rotateLeftB()
{
    /*rotate left wheel backward*/
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN2); // left motor
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN0);

    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN4); // right motor
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN5);
}


void rotateRightB()
{
    /*rotate Right wheel backward*/
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN2); // left motor
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0);

    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN4); // right motor
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN5);
}

void staticTurnLeft(){ //three point turn

    pwmConfig1.dutyCycle = 9312; //74% dc adjustment for accurate turn
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig1);
    pwmConfig.dutyCycle = 9312; //74% dc adjustment for accurate turn
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);

    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN2); //left motor
     GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0);

    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN4); // right motor
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN5);

    isForward = false;

}

void stopCar()
{
    /*stop both wheels*/
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN2); // left motor
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0);

    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN4); // right motor
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN5);
    isForward = false;
}

void threePointTurn()
    /*execute 3 point turn*/
{
    if (turnExecutionTime >= 1)
    {
        //stopCar();
        staticTurnLeft();
    }

    if (turnExecutionTime >= 4)
    {
        stopCar();
        //turnExecutionTime = 0;
//        isCompletimeCounter1ted=true;
    }
}
void turnLeft()
    /*execute 90 degree left point turn*/
{
    if (turnExecutionTime >= 1)
    {
        isCompleted=false;
        rotateRightF();
    }
    if (turnExecutionTime >= 3)
    {
        stopCar();
        //turnExecutionTime = 0;
        isCompleted=true;
    }
}

void turnRight()
    /*execute 90 degree right point turn*/
{
    if (turnExecutionTime >= 1)
    {
        isCompleted=false;
        rotateLeftF();
    }
    if (turnExecutionTime >= 3)
    {
        stopCar();
        //timeCounter1 = 0;
        isCompleted=true;
    }
}


void pidController(){
    double  pwmLeft = 0,
    pwmRight = 0;
    int     errLeft = 0,
    errRight = 0;

    /*
     * 1. Doing PID for each wheel
     * 2. run at 40% DC
     * 3. target = 75% of the encoder tick per sample. 11 * 0.75 = 8.25. round to nearest whole number = 8
     *                                                 33 * 0.75 = 24.75. round to nearest whole number = 25
     * 4. KP start point == 12500 / 25 = 500
     * 5. KI start point == 250 / 2 = 125
     * 6. KD start point == 500 / 2 = 250
     */

    //PID region (start)
    errLeft = target - notchesdetectedLeft;
    errRight = target - notchesdetectedRight;

    pwmLeft += (KP * errLeft) + (KI * sumErrLeft) + (KD * prevErrLeft) ;
    pwmRight += (KP * errRight) + (KI * sumErrRight) + (KD * prevErrRight) ;

    pwmLeft = pwmConfig1.dutyCycle + pwmLeft;
    pwmRight = pwmConfig.dutyCycle + pwmRight;

    pwmConfig1.dutyCycle = MAX(MIN(12500,pwmLeft),0);
    pwmConfig.dutyCycle = MAX(MIN(12500,pwmRight),0);

    pwmLeft = pwmConfig1.dutyCycle;
    pwmRight = pwmConfig.dutyCycle;

    printDouble(pwmLeft);
    printDouble(pwmRight);
    uPrintf("\n");

    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig1);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);


    sumErrLeft += errLeft;
    sumErrRight += errRight;
    prevErrLeft = errLeft;
    prevErrRight = errRight;
    notchesdetectedRight = 0;
    notchesdetectedLeft = 0;

    //PID region (End)
}

int main()
{

    notchesdetectedRight = 0;
    notchesdetectedLeft = 0;
    turnExecutionTime = 0;
    testNotch=0;
    isCompleted=false;


    initiatePortsForMotorDriver();
//    rotateForward();
    /*
     * 0: Stop
     * 1: Forward
     * 2: Left
     * 3: Right
     * 4: Three Point Turn
     */

    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION); // Set as  port as PWM output (Right Motor)
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION); // Set as  port as PWM output (Left Motor)

    /* Configuring Timer_A to have a period of approximately 80ms and an initial duty cycle of 10% of that (1000 ticks)  */
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig1);

    /*UART*/
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION); // Selecting P1.2 and P1.3 in UART mode
    UART_initModule(EUSCI_A0_BASE, &uartConfig);                                                                   // initializes UART, leaves it DISABLED
    UART_enableModule(EUSCI_A0_BASE);                                                                              // Enable UART module

    /*HC020K PHOTOELECTRIC ENCODER (right motor)*/
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN6); // Set interrupt for HC020K P2.6
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P2, GPIO_PIN6);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN6);

    /*HC020K PHOTOELECTRIC ENCODER (left motor)*/
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN7); // Set interrupt for HC020K P2.7
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P2, GPIO_PIN7);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN7);

    /*timer interrupt*/
    MAP_Timer_A_configureUpMode(TIMER_A1_BASE, &upConfigMotor); // Configuring Timer_A1 for Up Mode
    MAP_Interrupt_enableInterrupt(INT_TA1_0);

    /* Enabling interrupts and starting the watchdog timer */
    Interrupt_enableInterrupt(INT_PORT1);
    Interrupt_enableInterrupt(INT_PORT2);
    Interrupt_enableSleepOnIsrExit();
    Interrupt_enableMaster();                                            // enables master interrupt

    MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE); // Starting the Timer A1 in up mode

    /* Sleeping when not in use */
    while (1)
    {
        PCM_gotoLPM0();
    }
}

/* Port2 ISR*/ //wheel encoder
void PORT2_IRQHandler(void)
{

    uint32_t status;
    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P2);

    GPIO_clearInterruptFlag(GPIO_PORT_P2, status);

    if (status & GPIO_PIN6)
    {
    notchesdetectedRight++; //increment when notch detected
    }

    if (status & GPIO_PIN7)
    {
        notchesdetectedLeft++; //increment when notch detected
    }
}

//TIMER FOR MOTOR
void TA1_0_IRQHandler(void)
{

    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

    //   stopCar();
    //   threePointTurn();
    //   turnLeft();
    //   turnRight();

    turnExecutionTime++;

    if(turnExecutionTime == 1){
        rotateForward();
     }

    else if (turnExecutionTime == 61){ // stop
        stopCar();
    }


    if(isForward == true){
        pidController();
    }

}

void print(int num)
{
    char p[100];
    sprintf(p, "%d", num);
    uPrintf(p);
    uPrintf("\n\r");
}

void printDouble(double num)
{
    char p[100];
    sprintf(p, "%f", num);
    uPrintf(p);
    uPrintf("\n\r");
}

void uPrintf(unsigned char *TxArray)
{
    unsigned short i = 0;
    while (*(TxArray + i))
    {
        UART_transmitData(EUSCI_A0_BASE, *(TxArray + i)); // Write the character at the location specified by pointer
        i++;                                              // Increment pointer to point to the next character
    }
}

void EUSCIA0_IRQHandler(void)
{
    unsigned char a = 0;
    a = UART_receiveData(EUSCI_A0_BASE);
    UART_transmitData(EUSCI_A0_BASE, a);
}





