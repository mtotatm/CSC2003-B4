
#ifndef MAP_DIJKSTRA_H_
#define MAP_DIJKSTRA_H_

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

#define MIN_DISTANCE    10
#define TICKPERIOD      1000

#define ROW 10
#define COL 5


void ChangeDirection();
void PlotPosition();
void Initialize_mapping();
void Initialize_Navigation();
void Reached_Node();
void FinishedMapping_Comms();
void autoMapStop(); // auto stop code

void run_dijkstra();

//Motor
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
void initmotor();
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#endif
