#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_

bool checkFront(void);
bool checkLeft(void);
bool checkRight(void);
void initialise_Sensor(void);
static uint32_t getSensorTime(void);
static float getFrontSensorDistance(void);
static float getLeftSensorDistance(void);
static float getRightSensorDistance(void);
static void Delay(void);

// Interrupt handler
void TA2_0_IRQHandler(void);

#endif /* ULTRASONIC_H_ */
