/* DriverLib Includes */
#include "driverlib.h"

#define MIN_DISTANCE    15.0f
#define TICKPERIOD      1000
#define ULTRA_BUFFER_SIZE 10
#define EMA_WEIGHTING    0.18f // formula of 2/(10+1) 

uint32_t SR04IntTimes;
float frontdist[10];
float leftdist[10];
float rightdist[10];
bool frontDetected = false;
bool leftDetected = false;
bool rightDetected = false;



static void Delay(uint32_t loop) {
    volatile uint32_t i;

    for (i = 0 ; i < loop ; i++);
}

void initialise_Sensor(void) {
    /* Timer_A UpMode Configuration Parameter */
    const Timer_A_UpModeConfig upConfig = {
        TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_3,          // SMCLK/3 = 1MHz
        TICKPERIOD,                             // 1000 tick period
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,    // Enable CCR0 interrupt
        TIMER_A_DO_CLEAR                        // Clear value
    };

    int a = CS_getSMCLK();

    /* Configuring P5.6 as Trigger, 5.7 as Echo | Front sensor */
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN6);
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN6);
    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P6, GPIO_PIN7);

    /* Configuring P3.6 as Trigger, 2.7 as Echo | Left sensor */
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN6);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6);
    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P3, GPIO_PIN7);

    /* Configuring P4.6 as Trigger, 4.7 as Echo | Right sensor */
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN6);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN6);
    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P4, GPIO_PIN7);

    /* Configuring Timer_A0 for Up Mode */
    Timer_A_configureUpMode(TIMER_A2_BASE, &upConfig);

    /* Enabling interrupts and starting the timer */
    Interrupt_enableInterrupt(INT_TA2_0);

    Timer_A_clearTimer(TIMER_A2_BASE);
}

void TA2_0_IRQHandler(void) {
    /* Increment global variable (counter number of interrupt occurred) */
    SR04IntTimes++;

    /* Clear interrupt flag */
    Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
}

static uint32_t getSensorTime(void) {
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

static float getFrontSensorDistance(void) {
    uint32_t pulseduration = 0;
    float calculateddistance = 0;

    /* Generate 10us pulse at P5.6 */
    GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN6);
    Delay(30);
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN6);

    /* Wait for positive-edge */
    while(GPIO_getInputPinValue(GPIO_PORT_P6, GPIO_PIN7) == 0);

    /* Start Timer */
    SR04IntTimes = 0;
    Timer_A_clearTimer(TIMER_A2_BASE);
    Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_UP_MODE);

    /* Detects negative-edge */
    while(GPIO_getInputPinValue(GPIO_PORT_P6, GPIO_PIN7) == 1);

    /* Stop Timer */
    Timer_A_stopTimer(TIMER_A2_BASE);

    /* Obtain Pulse Width in microseconds */
    pulseduration = getSensorTime();

    /* Calculating distance in cm */
    calculateddistance = (float)pulseduration / 58.0f;

    //printf("%d\n", calculateddistance);
    return calculateddistance;
}

static float getLeftSensorDistance(void) {
    uint32_t pulseduration = 0;
    float calculateddistance = 0;

    /* Generate 10us pulse at P3.6 */
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6);
    Delay(30);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6);

    /* Wait for positive-edge */
    while(GPIO_getInputPinValue(GPIO_PORT_P3, GPIO_PIN7) == 0);

    /* Start Timer */
    SR04IntTimes = 0;
    Timer_A_clearTimer(TIMER_A2_BASE);
    Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_UP_MODE);

    /* Detects negative-edge */
    while(GPIO_getInputPinValue(GPIO_PORT_P3, GPIO_PIN7) == 1);

    /* Stop Timer */
    Timer_A_stopTimer(TIMER_A2_BASE);

    /* Obtain Pulse Width in microseconds */
    pulseduration = getSensorTime();

    /* Calculating distance in cm */
    calculateddistance = (float)pulseduration / 58.0f;

    //printf("%d\n", calculateddistance);
    return calculateddistance;
}

static float getRightSensorDistance(void) {
    uint32_t pulseduration = 0;
    float calculateddistance = 0;

    /* Generate 10us pulse at P4.6 */
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN6);
    Delay(30);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN6);

    /* Wait for positive-edge */
    while(GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN7) == 0);

    /* Start Timer */
    SR04IntTimes = 0;
    Timer_A_clearTimer(TIMER_A2_BASE);
    Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_UP_MODE);

    /* Detects negative-edge */
    while(GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN7) == 1);

    /* Stop Timer */
    Timer_A_stopTimer(TIMER_A2_BASE);

    /* Obtain Pulse Width in microseconds */
    pulseduration = getSensorTime();

    /* Calculating distance in cm */
    calculateddistance = (float)pulseduration / 58.0f;

    //printf("%d\n", calculateddistance);
    return calculateddistance;
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

//EMA fliter for front sensor (haven't tested yet)
float frontema(float dist){

    bool firstvalue = true;
    float previousvalue;
    float frontemavalue;

    if(firstvalue){   //Check if is the first value

        previousvalue = dist;
        return previousvalue;
    }
    else{

        frontemavalue = (dist*EMA_WEIGHTING) + (previousvalue*(1-EMA_WEIGHTING)); //EMA formula 
        return frontemavalue;
    
    }
}
//EMA fliter for left sensor (haven't tested yet)
float leftema(float dist){

    bool firstvalue = true;
    float previousvalue;
    float leftemavalue;

    if(firstvalue){   //Check if is the first value

        previousvalue = dist;
        return previousvalue;
    }
    else{

        leftemavalue = (dist*EMA_WEIGHTING) + (previousvalue*(1-EMA_WEIGHTING)); //EMA formula 
        return leftemavalue;
    
    }
}
//EMA fliter for right sensor (haven't tested yet)
float rightema(float dist){

    bool firstvalue = true;
    float previousvalue;
    float rightemavalue;

    if(firstvalue){   //Check if is the first value

        previousvalue = dist;
        return previousvalue;
    }
    else{

        rightemavalue = (dist*EMA_WEIGHTING) + (previousvalue*(1-EMA_WEIGHTING)); //EMA formula 
        return rightemavalue;
    
    }
}

bool checkFront(void) {
     //with SMA
    float frontfilteredDist = frontsma(getFrontSensorDistance());
    //without SMA
    //float frontfilteredDist = getFrontSensorDistance();
    //with EMA
    //float finalfrontfilteredDist = frontema(frontfilteredDist);
    return frontfilteredDist < MIN_DISTANCE;
}

bool checkLeft(void) {
    //with SMA
    float leftfilteredDist = leftsma(getLeftSensorDistance());
    //without SMA
    //float leftfilteredDist = getLeftSensorDistance();
    //with EMA
    //float finalleftfilteredDist = leftema(leftfilteredDist);
    return leftfilteredDist < MIN_DISTANCE;
}

bool checkRight(void) {
    //with SMA
    float rightfilteredDist = rightsma(getRightSensorDistance());
    //without SMA
    //float rightfilteredDist = getRightSensorDistance();
    //with EMA
    //float finalfrontfilteredDist = rightema(rightfilteredDist);
    return rightfilteredDist < MIN_DISTANCE;
}

uint32_t main(void) {
    /* Testing */
    initialise_Sensor();

    while(1)
    {
        Delay(3000);

        //For testing
        if (checkFront())
            printf("Front obstacle detected\n");
        if (checkLeft())
            printf("Left obstacle detected\n");
        if (checkRight())
            printf("Right obstacle detected\n");

     }
}
