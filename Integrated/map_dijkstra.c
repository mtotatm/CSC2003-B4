#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "map_dijkstra.h"
#include "uart_print.h"
#include <string.h>

#define MAXPATHPOINTS 100
#define NULLVALUE 0.123f
#define NORTH 0
#define SOUTH 1
#define EAST 2
#define WEST 3

#define DETECTED 1
#define NOT_DETECTED 0

#define FRONT 0
#define LEFT 1
#define RIGHT 2
#define BACK 3

#define MAPPING 0
#define NAVIGATION 1
#define WAITING 2

#define inf 100

#define ROW 10
#define COL 5
#define BUFFER_SIZE 128

#define target 8
#define KP 400
#define KI 195
#define KD 0.1

struct Point
{
    int x;
    int y;
};

int aa = 0;

int initialized_map = 0;
int pathpointnumber = 0;
int vertexnumber = 0;
int vertexsize;
int current_positionX = 0;
int current_positionY = 0;
struct Point pointarray[MAXPATHPOINTS];
int recurrenceArray[MAXPATHPOINTS]; // auto stop code
int current_direction;              // 0 = N , 1= S, 2= E, 3 = W
int mode = NAVIGATION;                 // 0 = mapping 1 = navigation 2= waiting
char ArrayOfString[ROW][COL];

struct Point previouspoint;

char userinput[10];

int startnode;
int endnode;
int vertexarray[] = {0, 1, 2};
int currentNodeGoal = 0;
// int isCompleted = 1;
int initialized_nav = 0;

char Buffer[BUFFER_SIZE];

void ChangeDirection();
void PlotPosition();
void Initialize_mapping();
void Initialize_Navigation();
void Reached_Node();
void FinishedMapping_Comms();
void autoMapStop(); // auto stop code
void PointCarDirectionToNextNode();

void array_fill(int *array, int len, int val);
int *dijkstra(int graph[][pathpointnumber], int n, int start, int end, int dist[]);
void run_dijkstra();

// Motor
uint32_t turnExecutionTime;
volatile int notchesdetectedRight = 0;
volatile int notchesdetectedLeft = 0;
volatile int prevErrLeft = 0;
volatile int prevErrRight = 0;
volatile int sumErrLeft = 0;
volatile int sumErrRight = 0;

bool isForward = true;
bool isCompleted = true;
uint32_t testNotch = 0;
int testNotchStatus = 0;

void initialise_Sensor();
void checkFront(void);
void checkLeft(void);
void checkRight(void);

uint32_t SR04IntTimes;
extern bool frontDetected = false;
extern bool leftDetected = false;
extern bool rightDetected = false;

volatile int distance = 0;
volatile int timerForSpeed = 0;
volatile int timerForSpeedInOneSec = 0;


// int main(){
//
//     pointarray[0].x = 0;
//     pointarray[0].y = 0;
//     pointarray[1].x = 50;
//     pointarray[1].y = 0;
//     pointarray[2].x = 50;
//     pointarray[2].y = 50;
//
//     current_direction = NORTH;
//
//     while(1){
//         //lcm
//     }
//
//     return 0;
// }

/*   ------------------------------------------------------ MOTOR CODE------------------------------------------------------------*/
/* Timer_A PWM Configuration Parameter <right motor> frequency = 1ms  */
Timer_A_PWMConfig pwmConfig =
    {
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_24,
        12500,
        TIMER_A_CAPTURECOMPARE_REGISTER_2,
        TIMER_A_OUTPUTMODE_RESET_SET,
        0};
/*Create a new timer for other wheel <left motor>*/
Timer_A_PWMConfig pwmConfig1 =
    {
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_24,
        12500,
        TIMER_A_CAPTURECOMPARE_REGISTER_1,
        TIMER_A_OUTPUTMODE_RESET_SET,
        0

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
    UART_Printf(EUSCI_A2_BASE, "Direction - rotate right \r\n");


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
    if (turnExecutionTime >= 0)
    {
        //stopCar();
        staticTurnLeft();
    }

    if (turnExecutionTime == 3)
    {
        stopCar();
        testNotch = 0;
        testNotchStatus = 0;
        isCompleted=true;
    }
}
void turnLeft()
    /*execute 90 degree left point turn*/
{
    if (turnExecutionTime >= 0)
    {
        isCompleted=false;
        rotateRightF();
    }
    if (turnExecutionTime == 2)
    {
        stopCar();
        testNotch = 0;
        testNotchStatus = 0;
        isCompleted=true;
        UART_Printf(EUSCI_A2_BASE, "Direction - Turn left \r\n");
    }
}

void turnRight()
    /*execute 90 degree right point turn*/
{
    if (turnExecutionTime >= 0)
    {
        isCompleted=false;
        rotateLeftF();
    }
    if (turnExecutionTime == 2)
    {
        stopCar();
        testNotch = 0;
        testNotchStatus = 0;
        isCompleted=true;
        UART_Printf(EUSCI_A2_BASE, "Direction - Turn right \r\n");


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

void initmotor(void)
{

    notchesdetectedRight = 0;
    notchesdetectedLeft = 0;

    turnExecutionTime = 0;
    testNotch = 0;
    isCompleted = false;

    initiatePortsForMotorDriver();
    /*
     * 0: Stop
     * 1: Forward
     * 2: Left
     * 3: Right
     * 4: Three Point Turn
     */
    // testNotchStatus = 1;

    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION); // Set as  port as PWM output (Right Motor)
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION); // Set as  port as PWM output (Left Motor)

    /* Configuring Timer_A to have a period of approximately 80ms and an initial duty cycle of 10% of that (1000 ticks)  */
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig1);
    //    Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfig);


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
    MAP_Timer_A_configureUpMode(TIMER_A1_BASE, &upConfigMotor); // Configuring Timer_A1 for Up Mode
    MAP_Interrupt_enableInterrupt(INT_TA1_0);

    /* Enabling interrupts and starting the watchdog timer */
    Interrupt_enableInterrupt(INT_PORT1);
    Interrupt_enableInterrupt(INT_PORT2);
    Interrupt_enableSleepOnIsrExit();

    MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE); // Starting the Timer A1 in up mode

    /* Sleeping when not in use */
    while (1)
    {
        PCM_gotoLPM0();
    }
}

/* Port2 ISR*/ // wheel encoder
void PORT2_IRQHandler(void)
{
    pointarray[0].x = 0;
    pointarray[0].y = 0;
    pointarray[1].x = 50;
    pointarray[1].y = 0;
    pointarray[2].x = 50;
    pointarray[2].y = 50;

    uint32_t status;
    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P2);

    GPIO_clearInterruptFlag(GPIO_PORT_P2, status);

    if (status & GPIO_PIN6)
    {
        notchesdetectedRight++;
        distance += 1;
        if (mode == MAPPING && testNotchStatus == 1)
        {
            // 0 = N , 1= S, 2= E, 3 = W
            switch (current_direction)
            {
            case NORTH:
                current_positionY++;
                break;
            case SOUTH:
                current_positionY--;
                break;
            case EAST:
                current_positionX++;
                break;
            case WEST:
                current_positionX--;
                break;
            }
        }
//        if (mode == NAVIGATION && testNotchStatus == 1)
//        {
//            // 0 = N , 1= S, 2= E, 3 = W
//            switch (current_direction)
//            {
//            case NORTH:
//                current_positionY++;
//                break;
//            case SOUTH:
//                current_positionY--;
//                break;
//            case EAST:
//                current_positionX++;
//                break;
//            case WEST:
//                current_positionX--;
//                break;
//            }
//        }
//        if(current_positionX == pointarray[vertexarray[currentNodeGoal]].x && current_positionY == pointarray[vertexarray[currentNodeGoal]].y){
//                    Reached_Node();
//                    PointCarDirectionToNextNode();
//       }
    }
    if (status & GPIO_PIN7)
    {
        notchesdetectedLeft++;
    }
}

// TIMER FOR MOTOR AND MAPPING
void TA1_0_IRQHandler(void)
{
    double pwmLeft = 0,
           pwmRight = 0;
    int errLeft = 0,
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
    

    //Send speed and distance
    UART_Printf(EUSCI_A2_BASE, "Speed- %i cm/s \r\n", cmPerSec);
    UART_Printf(EUSCI_A2_BASE, "Distance- %i cm \r\n", (distance));

    turnExecutionTime++;

//    frontDetected = checkFront();
//    leftDetected = checkLeft();
//    rightDetected = checkRight();

    /*
     * 0: Stop
     * 1: Forward
     * 2: Left
     * 3: Right
     * 4: Three Point Turn
     */

    //testNotchStatus = 1;

    if (testNotchStatus == 0) // 0: Stop
    {
        isForward = false;
        stopCar();
    }
    else if (testNotchStatus == 1) // 1: Forward
    {
        isForward = true;
        rotateForward();
    }
    else if (testNotchStatus == 2) // 2: Left
    {
        testNotch++;
        if (testNotch == 1)
        {
            turnExecutionTime = 0;
            isForward = false;
        }
        turnLeft();
    }
    else if (testNotchStatus == 3) // 3: Right
    {
        testNotch++;
        if (testNotch == 1)
        {
            turnExecutionTime = 0;
            isForward = false;
        }
        turnRight();
    }
    else if (testNotchStatus == 4) // 4: Three Point Turn
    {
        testNotch++;
        if (testNotch == 1)
        {
            turnExecutionTime = 0;
            isForward = false;
        }
        threePointTurn();
    }
    else
    {
        return;
    }

    if (isForward == true)
    {
        pidController();
    }



    //    if(isCompleted ==false && aa == 0){
    //            isCompleted = false;
    //            testNotchStatus = 2;
    //            aa++;
    //            return;
    //    }
    //    else if(isCompleted == true && aa == 1){
    //            isCompleted = false;
    //            testNotchStatus = 3;
    //            aa++;
    //            return;
    //    }
    //    else if(isCompleted == true && aa ==2){
    //        stopCar();
    //        testNotch =0;
    //        testNotchStatus =0;
    //        aa++;
    //    }
    //    else if(aa == 3 && isCompleted == true){
    //        testNotchStatus =1;
    //        aa++;
    //    }
    //    else if (isCompleted == true && aa <20){
    //        aa++;
    //    }
    //    else if(isCompleted == true && aa ==20){
    //        stopCar();
    //        testNotch =0;
    //        testNotchStatus =0;
    //        aa++;
    //    }
    //    else if(isCompleted == true && aa ==21){
    //        isCompleted = false;
    //        testNotchStatus = 2;
    //
    //
    //        aa++;
    //    }
    //    else if(isCompleted == true && aa ==22){
    //        isCompleted = false;
    //        testNotchStatus =3;
    //        aa++;
    //    }

    // mapping here
    if (mode == MAPPING && initialized_map == 0)
    { // mapping mode
        Initialize_mapping();
        initialized_map = 1;
        isCompleted = true;
    }
    if (mode == NAVIGATION && initialized_nav == 0)
    { // navigation mode
        Initialize_Navigation();
        PointCarDirectionToNextNode();
        initialized_nav = 1;
    }
    if (mode == WAITING)
    { // waiting for comms mode
        FinishedMapping_Comms();
    }

    if (mode == MAPPING)
    {
        if (isCompleted == true)
        {
            // wall is infront, no wall left or no wall right
            if (frontDetected == true || leftDetected == false || rightDetected == false)
            {
                if (aa == 0)
                {
                    stopCar();
                    testNotch = 0;
                    testNotchStatus = 0; // STOP

                    PlotPosition();
                    aa++;
                    return;
                }
                else if (aa == 1)
                {
                    isCompleted = false;
                    ChangeDirection();
                    aa = 0;
                    return;
                }
            }
        }
    }

    if (mode == NAVIGATION)
    {
        if (frontDetected == true || leftDetected == false || rightDetected == false)
        {
            if (aa == 0)
            {
                Reached_Node();
                aa++;
                return;
            }
            if (aa == 1)
            {
                isCompleted = false;
                PointCarDirectionToNextNode();
                aa = 0;
                return;
            }
        }
    }


    if (isCompleted == true && testNotchStatus != 1)
    {
        if (aa == 0)
        {
            stopCar();
            testNotch = 0;
            testNotchStatus = 0; // STOP
        }
        if (aa == 1)
        {
            testNotchStatus = 1;
        }
    }
}

/*----------------------------  MAPPING CODE -------------------------------------*/
void Initialize_mapping()
{
    current_positionX = 0;
    current_positionY = 0;
    current_direction = NORTH; // N
    previouspoint.x = current_positionX;
    previouspoint.y = current_positionY;
    vertexnumber = 0;
}

void PlotPosition()
{

    // int plotvertex = 1;
    int withinMargin = 0;

    //>5 <5 dont plot
    int i;
    for (i = 0; i < pathpointnumber; i++)
    {
        if (current_positionX < pointarray[i].x + 5 && current_positionX > pointarray[i].x - 5)
        {
            if (current_positionY < pointarray[i].y + 5 && current_positionY > pointarray[i].y - 5)
            {
                withinMargin++;
                recurrenceArray[i]++; // auto stop code
                i = pathpointnumber;
            }
            else
            {
                withinMargin = withinMargin;
            }
        }
        else
        {
            withinMargin = withinMargin;
        }
    }

    if (withinMargin == 0)
    {
        pathpointnumber++;
        pointarray[pathpointnumber].x = current_positionX;
        pointarray[pathpointnumber].y = current_positionY;
    }
}

void autoMapStop()
{

    int i;
    bool mapstop = true;

    for (i = 0; i < pathpointnumber; i++)
    {
        if (recurrenceArray[i] > 2 && pathpointnumber >= 10)
        {
            mapstop = true;
        }
        else
        {
            mapstop = false;
            i = pathpointnumber;
        }
    }

    if (mapstop == true)
    {
        FinishedMapping_Comms();
    }
}

size_t vSeparateSringByComma(char *string)
{
    const char *delims = ",\n";
    char *s = string;
    size_t n = 0, len;

    for (s = strtok(s, delims); s && n < ROW; s = strtok(NULL, delims))
        if ((len = strlen(s)) < COL)
            strcpy(ArrayOfString[n++], s);
        else
            fprintf(stderr, "error: '%s' exceeds COL - 1 chars.\n", s);

    return n;
}

void ChangeDirection()
{
    // if front sensor detected
    //    bool frontDetected=checkFront(); //front has value of 0
    //    bool leftDetected=checkLeft();  //left has value of 1
    //    bool rightDetected=checkRight(); //right has value of 2
    //                              //back has value of 3

    int randomdirection = 0;
    int selected_direction = 0;
    int previous_direction = current_direction;

    while (1)
    {
        randomdirection = rand() % 4;
        if (randomdirection == FRONT && frontDetected != true)
        {
            selected_direction = FRONT;

            break;
        }
        if (randomdirection == LEFT && leftDetected != true)
        {
            selected_direction = LEFT;
            break;
        }
        if (randomdirection == RIGHT && rightDetected != true)
        {
            selected_direction = RIGHT;
            break;
        }
        if (randomdirection == BACK && frontDetected == true && leftDetected == true && rightDetected == true)
        {
            selected_direction = BACK;
            break;
        }
    }

    if (current_direction == NORTH)
    {
        switch (selected_direction)
        {
        case FRONT:
            current_direction = NORTH;
            break;
        case BACK:
            current_direction = SOUTH;
            break;
        case LEFT:
            current_direction = WEST;
            break;
        case RIGHT:
            current_direction = EAST;
            break;
        }
    }
    else if (current_direction == SOUTH)
    {
        switch (selected_direction)
        {
        case FRONT:
            current_direction = SOUTH;
            break;
        case BACK:
            current_direction = NORTH;
            break;
        case LEFT:
            current_direction = EAST;
            break;
        case RIGHT:
            current_direction = WEST;
            break;
        }
    }
    else if (current_direction == EAST)
    {
        switch (selected_direction)
        {
        case FRONT:
            current_direction = EAST;
            break;
        case BACK:
            current_direction = WEST;
            break;
        case LEFT:
            current_direction = NORTH;
            break;
        case RIGHT:
            current_direction = SOUTH;
            break;
        }
    }
    else if (current_direction == WEST)
    {
        switch (selected_direction)
        {
        case FRONT:
            current_direction = WEST;
            break;
        case BACK:
            current_direction = EAST;
            break;
        case LEFT:
            current_direction = SOUTH;
            break;
        case RIGHT:
            current_direction = NORTH;
            break;
        }
    }

    switch (selected_direction)
    {
    case FRONT:
        break;
    case BACK:
        testNotchStatus = 4; // THREE_POINT_TURN
        break;
    case LEFT:
        testNotchStatus = 2; // TURN_LEFT
        break;
    case RIGHT:
        testNotchStatus = 3; // TURN_RIGHT
        break;
    }
}

void Initialize_Navigation()
{
    // current direction = uart sets
    current_positionX = pointarray[vertexarray[0]].x;
    current_positionY = pointarray[vertexarray[0]].y;
    currentNodeGoal = 1;
}

void PointCarDirectionToNextNode()
{
    int i = 0;
    i = currentNodeGoal;
    if (current_positionX < pointarray[vertexarray[i]].x)
    { // GO EAST
        switch (current_direction)
        {
        case NORTH:
            testNotchStatus = 3; // TURN_RIGHT
            current_direction = EAST;
            break;
        case SOUTH:
            testNotchStatus = 2; // TURN_LEFT
            current_direction = EAST;
            break;
        case WEST:
            testNotchStatus = 4; // THREE_POINT_TURN
            current_direction = EAST;
            break;
        case EAST:
            break;
        }
    }
    else if (current_positionX > pointarray[vertexarray[i]].x)
    { // GO WEST
        switch (current_direction)
        {
        case NORTH:
            testNotchStatus = 2; // TURN_LEFT
            current_direction = WEST;
            break;
        case SOUTH:
            testNotchStatus = 3; // TURN_RIGHT
            current_direction = WEST;
            break;
        case EAST:
            testNotchStatus = 4; // THREE_POINT_TURN
            current_direction = WEST;
            break;
        case WEST:
            break;
        }
    }
    else if (current_positionY < pointarray[vertexarray[i]].y)
    { // GO NORTH
        switch (current_direction)
        {
        case WEST:
            testNotchStatus = 3; // TURN_RIGHT
            current_direction = NORTH;
            break;
        case SOUTH:
            testNotchStatus = 4; // THREE_POINT_TURN
            current_direction = NORTH;
            break;
        case EAST:
            testNotchStatus = 2; // TURN_LEFT
            current_direction = NORTH;
            break;
        case NORTH:
            break;
        }
    }
    else if (current_positionY > pointarray[vertexarray[i]].y)
    { // GO SOUTH
        switch (current_direction)
        {
        case WEST:
            testNotchStatus = 2; // TURN_LEFT
            current_direction = SOUTH;
            break;
        case NORTH:
            testNotchStatus = 4; // THREE_POINT_TURN
            current_direction = SOUTH;
            break;
        case EAST:
            testNotchStatus = 3; // TURN_RIGHT
            current_direction = SOUTH;
            break;
        case SOUTH:
            break;
        }
    }
}

void Reached_Node()
{
    stopCar();
    testNotch = 0;
    testNotchStatus = 0;
    current_positionX = pointarray[vertexarray[currentNodeGoal]].x;
    current_positionY = pointarray[vertexarray[currentNodeGoal]].y;
    currentNodeGoal++;
}

void FinishedMapping_Comms()
{
    mode = 2;
    if (mode == MAPPING)
    {
        // temp pointarray for testing
        pointarray[0].x = 0;
        pointarray[0].y = 0;
        pointarray[1].x = 50;
        pointarray[1].y = 0;
        pointarray[2].x = 50;
        pointarray[2].y = 50;
        // size is pre-defined as 2 for testing purposes
        // will be changed to pathpointnumber <--
        int size;
        int i;

        for (size = 2, i = 0; i <= size; i++)
        {
            char x[100];
            char y[100];
            char z[100];
            sprintf(x, "%d", i + 1);
            sprintf(y, "%d", pointarray[i].x);
            sprintf(z, "%d", pointarray[i].y);
            UART_Printf(EUSCI_A2_BASE, "Mapping-Node%s: x=%s, y=%s \r\n", x, y, z);
        }

        // ask for input
        UART_Printf(EUSCI_A2_BASE, "Message - Please input a start and end node");
        mode = WAITING;
    }
    else if (mode == WAITING)
    {

        // if comms data found
        UART_Gets(EUSCI_A2_BASE, Buffer, BUFFER_SIZE);
        UART_Printf(EUSCI_A0_BASE, "%s", Buffer);

        // char string[] = "123,4,5";
        size_t n = vSeparateSringByComma(Buffer);

        startnode = atoi(ArrayOfString[0]);
        endnode = atoi(ArrayOfString[1]);
        current_direction = atoi(ArrayOfString[2]);

        UART_Printf(EUSCI_A0_BASE, "%i, %i, %i", startnode, endnode, current_direction);
        // char x[100];
        // sprintf(x,"%d", startnode);
        // UART_Printf(EUSCI_A0_BASE,"%s",x);

        mode = NAVIGATION;
    }
}

// int Polling(){
//     if(mode == MAPPING && initialized_map == 0){//mapping mode
//         Initialize_mapping();
//         initialized_map = 1;
//     }
//     if(mode == NAVIGATION && initialized_nav == 0){ //navigation mode
//         Initialize_Navigation();
//         PointCarDirectionToNextNode();
//         initialized_nav = 1;
//     }
//     if(mode == WAITING){ //waiting for comms mode
//         FinishedMapping_Comms();
//     }
//
//     if(isCompleted == 1 && testNotchStatus != 1){
//         testNotchStatus = 1; //move forward
//         isCompleted = 0;
//     }
//
// }

// int Interrupt_1(){ //when more than 1 path detected
//     if(mode == MAPPING){
//         testNotchStatus=0;//STOP
//         PlotPosition();
//         ChangeDirection();
//     }else if(mode == NAVIGATION){
//         Reached_Node();
//         PointCarDirectionToNextNode();
//     }
//
// }

// int Interrupt_2(){ //when wall is detected in front
//
//     if(mode == MAPPING){
//         testNotchStatus=0;//STOP
//         PlotPosition();
//         ChangeDirection();
//     }else if(mode == NAVIGATION){
//         Reached_Node();
//         PointCarDirectionToNextNode();
//     }
//
// }

// int Interrupt_3(){ //every wheel spin wheel encoder
//
//
// }

/*
  Dijkstra Algo - Array Fill
*/
void array_fill(int *array, int len, int val)
{
    int i;
    for (i = 1; i < len; i++)
    {
        array[i] = val;
    }
}

/*
  Dijkstra Algo - Logic
*/
int *dijkstra(int graph[][pathpointnumber], int n, int start, int end, int dist[])
{
    int *path = (int *)malloc(sizeof(int) * n);
    int *shortest = (int *)malloc(sizeof(int) * n);
    int *mark = (int *)malloc(sizeof(int) * n);
    int min, v, i, j;
    static int vertexarray[MAXPATHPOINTS];
    static int store[2];
    array_fill(mark, n, 0);
    array_fill(dist, n, inf);

    for (i = 0; i < n; i++)
    {
        dist[i] = graph[start][i];
        if (i != start && dist[i] < inf)
            path[i] = start;
        else
            path[i] = -1;
    }
    mark[start] = 1;
    while (1)
    {
        min = inf;
        v = -1;
        // find smallest dist
        for (i = 0; i < n; i++)
        {
            if (!mark[i])
            {
                if (dist[i] < min)
                {
                    min = dist[i];
                    v = i;
                }
            }
        }
        if (v == -1)
            break; // if there are no more shortest path
        // update shortest path
        mark[v] = 1;
        for (i = 0; i < n; i++)
        {
            if (!mark[i] &&
                graph[v][i] != inf &&
                dist[v] + graph[v][i] < dist[i])
            {
                dist[i] = dist[v] + graph[v][i];
                path[i] = v;
            }
        }
    }

    // generate path
    printf("start\t\tend\t\tcost\t\tpath \n");
    for (i = 0; i < 2; i++)
    {
        i = end;
        if (i == start)
            continue;
        array_fill(shortest, n, 0);
        printf("%d\t\t\t", start);
        printf("%d\t\t\t", i);
        printf("%d\t\t\t", dist[i]);
        int k = 0;
        shortest[k] = i;
        while (path[shortest[k]] != start)
        {
            k++;
            shortest[k] = path[shortest[k - 1]];
        }
        k++;
        shortest[k] = start;
        for (j = k; j > 0; j--)
        {
            printf("%d->", shortest[j]);
            vertexarray[j] = shortest[j];
            vertexsize++;
        }
        printf("%d\n", shortest[0]);
        vertexarray[0] = shortest[0];
        vertexsize++;
    }
    // Free memory after dynamic allocation
    free(path);
    free(shortest);
    free(mark);

    /*
      - store[0] : returns an int[] array that contains the route ( eg. {0,9,3} )
      - store[1] : returns size of vertexarray ^
    */
    store[0] = vertexarray;
    store[1] = vertexsize;

    return store;
}

/*
  Dijkstra Algo - Run algo
*/
void run_dijkstra()
{

    struct Point arr[pathpointnumber];
    int i;
    for (i = 0; i < pathpointnumber; i++)
    {

        arr[i].x = pointarray[i].x;
        arr[i].y = pointarray[i].y;
    }

    // temp graph
    int G[pathpointnumber][pathpointnumber];

    for (i = 0; i < pathpointnumber; i++)
    {
        printf("%d: %d %d\n", i, arr[i].x, arr[i].y);
        int j;
        for (j = 0; j < pathpointnumber; j++)
        {
            // If it is own node, set cost to 0
            if (arr[j].x == arr[i].x && arr[j].y == arr[i].y)
            {
                G[i][j] = 0;
            }
            // Check against the x-axis
            else if (arr[j].x == arr[i].x)
            {
                // Get the cost by subtracting the y-axis
                int cost = arr[j].y - arr[i].y;
                // If cost becomes negative, we abs it
                if (cost < 0)
                {
                    cost = abs(cost);
                }
                G[i][j] = cost;
            }
            // Check against the y-axis
            else if (arr[j].y == arr[i].y)
            {
                // Get the cost by subtracting the x-axis
                int cost = arr[j].x - arr[i].x;
                // If cost becomes negative, we abs it
                if (cost < 0)
                {
                    cost = abs(cost);
                }
                G[i][j] = cost;
            }
            // Nodes that are not possible to connect will be inf.
            else
            {
                G[i][j] = inf;
            }
        }
    }

    // check graph
    /* for (int i = 0; i < SIZE; ++i)
      for (int j = 0; j < SIZE; ++j) {
        printf("%d   ", G[i][j]);
        if (j == j - 1) {
          printf("\n\n");
        }
      } */

    int dist[pathpointnumber];
    /* dijkstra(W,V_SIZE,0,3,dist); */

    // generating shortest route
    int start, end;
    int *getRoute;

    // User will input start and end node and the path will be generated
    printf("Enter start node:\n");
    scanf("%d", &start);
    printf("Enter end node:\n");
    scanf("%d", &end);

    getRoute = dijkstra(G, pathpointnumber, start, end, dist);

    /*
      ~ Testing ~
      ***
      calling func() dijkstra will return 2 params : vertexarray & vertexsize
      ***
      vertexarray -> contain nodes of the shortest path, stored in an int[] array
      vertexsize -> size of vertexarray
    */

    int *getRoutePath = getRoute[0];
    printf("\nroute length: %d\n", getRoute[1]);

    for (i = getRoute[1] - 1; i >= 0; i--)
    {
        printf("%d", getRoutePath[i]);
    }

    return 0;
}

/*********************************************** ULTRASONIC CODE *********************************************************************/
static void Delay(uint32_t loop)
{
    volatile uint32_t i;

    for (i = 0; i < loop; i++)
        ;
}

float frontdist[10];
float leftdist[10];
float rightdist[10];

/* Timer_A UpMode Configuration Parameter */
const Timer_A_UpModeConfig upConfigUltra = {
    TIMER_A_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
    TIMER_A_CLOCKSOURCE_DIVIDER_3,      // SMCLK/3 = 1MHz
    TICKPERIOD,                         // 1000 tick period
    TIMER_A_TAIE_INTERRUPT_DISABLE,     // Disable Timer interrupt
    TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE, // Enable CCR0 interrupt
    TIMER_A_DO_CLEAR                    // Clear value
};

void initialise_Sensor(void)
{

    int a = CS_getSMCLK();

    /* Configuring P6.6 as Trigger, 6.7 as Echo */
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN6);
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN6);
    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P6, GPIO_PIN7);

    /* Configuring P3.6 as Trigger, 3.7 as Echo */
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN6);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6);
    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P3, GPIO_PIN7);

    /* Configuring P4.6 as Trigger, 4.7 as Echo */
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN6);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN6);
    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P4, GPIO_PIN7);

    /* Configuring Timer_A0 for Up Mode */
    Timer_A_configureUpMode(TIMER_A2_BASE, &upConfigUltra);

    /* Enabling interrupts and starting the timer */
    Interrupt_enableInterrupt(INT_TA2_0);

    MAP_Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_UP_MODE);

    //Timer_A_clearTimer(TIMER_A2_BASE);
}

// ultra sensor
void TA2_0_IRQHandler(void)
{
    /* Increment global variable (count number of interrupt occurred) */
    SR04IntTimes++;

    checkFront();
    checkLeft();
    checkRight();
    


    /* Clear interrupt flag */
    Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

//    if (!frontDetected)
//    {
//        //UART_Printf(EUSCI_A0_BASE, "front");
//
//        testNotchStatus =1;
//    }
//    else if(frontDetected)
//    {
//        if (!leftDetected && rightDetected)
//        {
//            //UART_Printf(EUSCI_A0_BASE, "left");
//
//            testNotchStatus =2;
//        }
//        else if (!rightDetected && leftDetected)
//        {
//            //UART_Printf(EUSCI_A0_BASE, "right");
//
//            testNotchStatus =3;
//        }
//    }
//    else if (frontDetected && leftDetected && rightDetected)
//    {
//        testNotchStatus =0;
//    }

    UART_Printf(EUSCI_A2_BASE, "Notch-%i \r\n",testNotchStatus);
}

static uint32_t getSensorTime(void)
{
    uint32_t pulsetime = 0;

    /* Number of times the interrupt occurred (1 interrupt = 1000 ticks)    */
    pulsetime = SR04IntTimes * TICKPERIOD;

    /* Number of ticks (between 1 to 999) before the interrupt could occur */
    pulsetime += Timer_A_getCounterValue(TIMER_A2_BASE);

    /* Clear Timer */
    Timer_A_clearTimer(TIMER_A2_BASE);

    Delay(3000);

    return pulsetime;
}

static float getFrontSensorDistance(void)
{
    uint32_t pulseduration = 0;
    float calculateddistance = 0;

    /* Generate 10us pulse at P5.6 */
    GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN6);
    Delay(30);
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN6);

    /* Wait for positive-edge */
    while (GPIO_getInputPinValue(GPIO_PORT_P6, GPIO_PIN7) == 0)
        ;

    /* Start Timer */
    SR04IntTimes = 0;
    Timer_A_clearTimer(TIMER_A2_BASE);
    Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_UP_MODE);

    /* Detects negative-edge */
    while (GPIO_getInputPinValue(GPIO_PORT_P6, GPIO_PIN7) == 1)
        ;

    /* Stop Timer */
    Timer_A_stopTimer(TIMER_A2_BASE);

    /* Obtain Pulse Width in microseconds */
    pulseduration = getSensorTime();

    /* Calculating distance in cm */
    calculateddistance = (float)pulseduration / 58.0f;

    // printf("%d\n", calculateddistance);
    return calculateddistance;
}

static float getLeftSensorDistance(void)
{
    uint32_t pulseduration = 0;
    float calculateddistance = 0;

    /* Generate 10us pulse at P3.6 */
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6);
    Delay(30);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6);

    /* Wait for positive-edge */
    while (GPIO_getInputPinValue(GPIO_PORT_P3, GPIO_PIN7) == 0)
        ;

    /* Start Timer */
    SR04IntTimes = 0;
    Timer_A_clearTimer(TIMER_A2_BASE);
    Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_UP_MODE);

    /* Detects negative-edge */
    while (GPIO_getInputPinValue(GPIO_PORT_P3, GPIO_PIN7) == 1)
        ;

    /* Stop Timer */
    Timer_A_stopTimer(TIMER_A2_BASE);

    /* Obtain Pulse Width in microseconds */
    pulseduration = getSensorTime();

    /* Calculating distance in cm */
    calculateddistance = (float)pulseduration / 58.0f;

    // printf("%d\n", calculateddistance);
    return calculateddistance;
}

static float getRightSensorDistance(void)
{
    uint32_t pulseduration = 0;
    float calculateddistance = 0;

    /* Generate 10us pulse at P4.6 */
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN6);
    Delay(30);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN6);

    /* Wait for positive-edge */
    while (GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN7) == 0)
        ;

    /* Start Timer */
    SR04IntTimes = 0;
    Timer_A_clearTimer(TIMER_A2_BASE);
    Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_UP_MODE);

    /* Detects negative-edge */
    while (GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN7) == 1)
        ;

    /* Stop Timer */
    Timer_A_stopTimer(TIMER_A2_BASE);

    /* Obtain Pulse Width in microseconds */
    pulseduration = getSensorTime();

    /* Calculating distance in cm */
    calculateddistance = (float)pulseduration / 58.0f;

    // printf("%d\n", calculateddistance);
    return calculateddistance;
}

// SMA for right sensor
float rightsma(float value) {
    static int counter = 0;
    static int position = 0;
    float sum = 0;
    if(value < 200.0){ //Only take in value below 200 due to random spike above 200 when testing
        rightdist[position] = value;
        if (counter != 10)
        {
            counter += 1;
        }

        if (position == 9) { // To constantly replace values in the array to get the latest 10
            position = 0;
        }
        else {
            position += 1;
        }
    }
    int i;
    for (i = 0; i < 10; i++) {
        sum += rightdist[i];
    }


    return (sum/counter); // get the average distance
}

// SMA for left sensor
float leftsma(float value) {
    static int counter = 0;
    static int position = 0;
    float sum = 0;
    if(value < 200.0){//Only take in value below 200 due to random spike above 200 when testing
        leftdist[position] = value;
        if (counter != 10)
        {
            counter += 1;
        }

        if (position == 9) { // To constantly replace values in the array to get the latest 10
            position = 0;
        }
        else {
            position += 1;
        }
    }

    int i;
    for (i = 0; i < 10; i++) {
        sum += leftdist[i];
    }


    return (sum/counter); // get the average distance
}
// SMA for front sensor
float frontsma(float value) {
    static int counter = 0;
    static int position = 0;
    float sum = 0;
    if(value < 200.0){//Only take in value below 200 due to random spike above 200 when testing
      frontdist[position] = value;
      if (counter != 10)
      {
        counter += 1;
      }
      if (position == 9) { // To constantly replace values in the array to get the latest 10
        position = 0;
      }
      else {
        position += 1;
      }
    }
    int i;
    for (i = 0; i < 10; i++) {
        sum += frontdist[i];
    }


    return (sum/counter); // get the average distance
}



void checkFront(void)
{
    frontDetected = (getFrontSensorDistance() < MIN_DISTANCE);
    float filteredDist = frontsma(getFrontSensorDistance());
}

void checkLeft(void)
{
    leftDetected = (getLeftSensorDistance() < MIN_DISTANCE);
    float filteredDist = leftsma(getLeftSensorDistance());
}

void checkRight(void)
{
    rightDetected = (getRightSensorDistance() < MIN_DISTANCE);
    float filteredDist = rightsma(getRightSensorDistance());
}
