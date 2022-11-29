/*
 * motor.h
 *
 *  Created on: 25 Nov 2022
 *      Author: Calvert
 */

#ifndef MOTOR_H_
#define MOTOR_H_

int runMotor(void);

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


void PORT2_IRQHandler(void);
void TA1_0_IRQHandler(void);

#endif /* MOTOR_H_ */
