#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

//#include "map_dijkstra.h"
//#include "uart_print.h"
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

struct Point {
    int x;
    int y;
};


int aa =0;

int initialized_map = 0;
int pathpointnumber = 0;
int vertexnumber= 0;
int vertexsize;
int current_positionX=0;
int current_positionY=0;
struct Point pointarray[MAXPATHPOINTS];
int recurrenceArray[MAXPATHPOINTS]; //auto stop code
int current_direction;// 0 = N , 1= S, 2= E, 3 = W
int mode = MAPPING; //0 = mapping 1 = navigation 2= waiting
char ArrayOfString[ROW][COL];

struct Point previouspoint;

char userinput[10];

int startnode;
int endnode;
int vertexarray[] = {1,2,3};
int currentNodeGoal = 0;
//int isCompleted = 1;
int initialized_nav =0;


char Buffer[BUFFER_SIZE];

void ChangeDirection();
void PlotPosition();
void Initialize_mapping();
void Initialize_Navigation();
void Reached_Node();
void FinishedMapping_Comms();
void autoMapStop(); // auto stop code
void PointCarDirectionToNextNode();



void array_fill(int * array, int len, int val);
int* dijkstra(int graph[][pathpointnumber],int n,int start, int end, int dist[]);
void run_dijkstra();


bool isCompleted=true;


extern bool frontDetected = false;
extern bool leftDetected = false;
extern bool rightDetected = false;

int testNotchStatus=1;
/*
* 0: Stop
* 1: Forward
* 2: Left
* 3: Right
* 4: Three Point Turn
*/

void PORT2_IRQHandler(void)
{

    uint32_t status;
    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P2);

    GPIO_clearInterruptFlag(GPIO_PORT_P2, status);

    if (status & GPIO_PIN6)
    {
        notchesdetectedRight++;
        if(mode == MAPPING && testNotchStatus == 1){
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

    }
    if (status & GPIO_PIN7)
    {
        notchesdetectedLeft++;
    }
}




void TA1_0_IRQHandler(void)
{





        //mapping here
        if(mode == MAPPING && initialized_map == 0){//mapping mode
            Initialize_mapping();
            initialized_map = 1;
            isCompleted =true;
        }
        if(mode == NAVIGATION && initialized_nav == 0){ //navigation mode
            run_dijkstra();
            Initialize_Navigation();
            PointCarDirectionToNextNode();
            initialized_nav = 1;
        }
        if(mode == WAITING){ //waiting for comms mode
            FinishedMapping_Comms();
        }





        if(mode == MAPPING){
            if(isCompleted == true){
                //wall is infront, no wall left or no wall right
                if(frontDetected == true || leftDetected == false || rightDetected == false ){
                    if(aa ==0){
                        stopCar();
                        //testNotch =0;
                        testNotchStatus =0;//STOP

                        PlotPosition();
                        aa++;
                        return;
                    }
                    else if(aa ==1){
                        isCompleted = false;
                        ChangeDirection();
                        aa = 0;
                        return;
                    }
                }
            }
        }

        if(mode == NAVIGATION){
            if(frontDetected == true || leftDetected == false || rightDetected == false){
                if(aa == 0){
                    Reached_Node();
                    aa++;
                    return;
                }
                if(aa == 1){
                    isCompleted = false;
                    PointCarDirectionToNextNode();
                    aa = 0;
                    return;
                }
            }
        }

        if(isCompleted == true && testNotchStatus != 1){
            if(aa == 0){
                stopCar();
                testNotch =0;
                testNotchStatus =0;//STOP
            }
            if(aa == 1){
                testNotchStatus = 1;
            }
        }




}



void Initialize_mapping(){
    current_positionX = 0;
    current_positionY = 0;
    current_direction = NORTH; //N
    previouspoint.x = current_positionX;
    previouspoint.y = current_positionY;
    vertexnumber = 0;


}

void PlotPosition(){

    //int plotvertex = 1;
    int withinMargin = 0;

    //>5 <5 dont plot
    int i;
    for(i = 0; i < pathpointnumber; i++){
        if(current_positionX < pointarray[i].x + 5 && current_positionX > pointarray[i].x - 5){
            if(current_positionY < pointarray[i].y + 5 && current_positionY > pointarray[i].y - 5){
                withinMargin++;
                recurrenceArray[i]++; // auto stop code
                i = pathpointnumber;
            }
            else{
                withinMargin = withinMargin;
            }
        }
        else{
            withinMargin = withinMargin;
        }
    }

    if(withinMargin == 0){
        pathpointnumber++;
        pointarray[pathpointnumber].x = current_positionX;
        pointarray[pathpointnumber].y = current_positionY;
    }
}

void autoMapStop(){

  int i;
  bool mapstop = true;

  for(i = 0; i < pathpointnumber; i++){
      if(recurrenceArray[i] > 2 && pathpointnumber >= 10){
          mapstop = true;
      }
      else{
          mapstop = false;
          i = pathpointnumber;
      }
  }

  if(mapstop == true){
      FinishedMapping_Comms();
  }
}

size_t vSeparateSringByComma(char *string) {
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

void ChangeDirection(){
    //if front sensor detected
//    bool frontDetected=checkFront(); //front has value of 0
//    bool leftDetected=checkLeft();  //left has value of 1
//    bool rightDetected=checkRight(); //right has value of 2
//                              //back has value of 3

    int randomdirection=0;
    int selected_direction=0;
    int previous_direction = current_direction;

    while(1){
        randomdirection = rand() %4;
        if(randomdirection == FRONT && frontDetected != true){
            selected_direction = FRONT;
            break;
        }
        if(randomdirection == LEFT && leftDetected != true){
            selected_direction = LEFT;
            break;
        }
        if(randomdirection == RIGHT && rightDetected != true){
            selected_direction = RIGHT;
            break;
        }
        if(randomdirection == BACK && frontDetected == true && leftDetected == true && rightDetected == true){
            selected_direction = BACK;
            break;
        }
    }

    if(current_direction == NORTH){
        switch(selected_direction){
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
    else if(current_direction == SOUTH){
        switch(selected_direction){
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
    else if(current_direction == EAST){
        switch(selected_direction){
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
    else if(current_direction == WEST){
        switch(selected_direction){
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

    switch(selected_direction){
        case FRONT:
            break;
        case BACK:
            testNotchStatus=4;//THREE_POINT_TURN
            break;
        case LEFT:
            testNotchStatus=2;//TURN_LEFT
            break;
        case RIGHT:
            testNotchStatus=3;//TURN_RIGHT
            break;
    }


}

void Initialize_Navigation(){
    //current direction = uart sets
    current_positionX = pointarray[vertexarray[0]].x;
    current_positionY = pointarray[vertexarray[0]].y;
    currentNodeGoal = 1;
}

void PointCarDirectionToNextNode(){
    int i = 0;
    i = currentNodeGoal;
    if(current_positionX < pointarray[vertexarray[i]].x){ // GO EAST
        switch(current_direction){
            case NORTH:
                testNotchStatus=3;//TURN_RIGHT
                current_direction = EAST;
                break;
            case SOUTH:
                testNotchStatus=2;//TURN_LEFT
                current_direction = EAST;
                break;
            case WEST:
                testNotchStatus=4;//THREE_POINT_TURN
                current_direction = EAST;
                break;
            case EAST:
                break;
        }
    }
    else if(current_positionX > pointarray[vertexarray[i]].x){ //GO WEST
        switch(current_direction){
            case NORTH:
                testNotchStatus=2;//TURN_LEFT
                current_direction = WEST;
                break;
            case SOUTH:
                testNotchStatus=3;//TURN_RIGHT
                current_direction = WEST;
                break;
            case EAST:
                testNotchStatus=4;//THREE_POINT_TURN
                current_direction = WEST;
                break;
            case WEST:
                break;
        }
    }
    else if(current_positionY < pointarray[vertexarray[i]].y){ //GO NORTH
        switch(current_direction){
            case WEST:
                testNotchStatus=3;//TURN_RIGHT
                current_direction = NORTH;
                break;
            case SOUTH:
                testNotchStatus=4;//THREE_POINT_TURN
                current_direction = NORTH;
                break;
            case EAST:
                testNotchStatus=2;//TURN_LEFT
                current_direction = NORTH;
                break;
            case NORTH:
                break;
        }
    }
    else if(current_positionY > pointarray[vertexarray[i]].y){ //GO SOUTH
        switch(current_direction){
            case WEST:
                testNotchStatus=2;//TURN_LEFT
                current_direction = SOUTH;
                break;
            case NORTH:
                testNotchStatus=4;//THREE_POINT_TURN
                current_direction = SOUTH;
                break;
            case EAST:
                testNotchStatus=3;//TURN_RIGHT
                current_direction = SOUTH;
                break;
            case SOUTH:
                break;
        }
    }
}

void Reached_Node(){
    stopCar();
    testNotch =0;
    testNotchStatus =0;
    current_positionX = pointarray[vertexarray[currentNodeGoal]].x;
    current_positionY = pointarray[vertexarray[currentNodeGoal]].y;
    if(currentNodeGoal == (sizeof(vertexarray)/sizeof(vertexarray[0]) - 1)){
        currentNodeGoal++;
    }

}

void FinishedMapping_Comms(){
    mode = 2;
    if(mode == MAPPING){
        // temp pointarray for testing
        // pointarray[0].x = 0;
        // pointarray[0].y = 10;
        // pointarray[1].x = 10;
        // pointarray[1].y = 10;
        // pointarray[2].x = 20;
        // pointarray[2].y = 20;
        // size is pre-defined as 2 for testing purposes
        // will be changed to pathpointnumber <--
        int size;
        int i;

        for (size = 2, i = 0; i <= size; i++) {
            char x[100];
            char y[100];
            char z[100];
            sprintf(x,"%d", i+1);
            sprintf(y,"%d", pointarray[i].x);
            sprintf(z,"%d", pointarray[i].y);
            UART_Printf(EUSCI_A2_BASE, "Mapping-Node%s: x=%s, y=%s \r\n",x, y, z);

        }

        //ask for input
        UART_Printf(EUSCI_A2_BASE, "Please input a start and end node");
        mode = WAITING;

    }
    else if (mode == WAITING){

        //if comms data found
        UART_Gets(EUSCI_A2_BASE,Buffer,BUFFER_SIZE);
        UART_Printf(EUSCI_A0_BASE,"%s",Buffer);

        // char string[] = "123,4,5";
        size_t n = vSeparateSringByComma(Buffer);

        startnode = atoi(ArrayOfString[0]);
        endnode = atoi(ArrayOfString[1]);
        current_direction = atoi(ArrayOfString[2]);

        UART_Printf(EUSCI_A0_BASE,"%i, %i, %i",startnode,endnode,current_direction);
        //char x[100];
        //sprintf(x,"%d", startnode);
        //UART_Printf(EUSCI_A0_BASE,"%s",x);

        mode = NAVIGATION;
    }


}

//int Polling(){
//    if(mode == MAPPING && initialized_map == 0){//mapping mode
//        Initialize_mapping();
//        initialized_map = 1;
//    }
//    if(mode == NAVIGATION && initialized_nav == 0){ //navigation mode
//        Initialize_Navigation();
//        PointCarDirectionToNextNode();
//        initialized_nav = 1;
//    }
//    if(mode == WAITING){ //waiting for comms mode
//        FinishedMapping_Comms();
//    }
//
//    if(isCompleted == 1 && testNotchStatus != 1){
//        testNotchStatus = 1; //move forward
//        isCompleted = 0;
//    }
//
//}




//int Interrupt_1(){ //when more than 1 path detected
//    if(mode == MAPPING){
//        testNotchStatus=0;//STOP
//        PlotPosition();
//        ChangeDirection();
//    }else if(mode == NAVIGATION){
//        Reached_Node();
//        PointCarDirectionToNextNode();
//    }
//
//}

//int Interrupt_2(){ //when wall is detected in front
//
//    if(mode == MAPPING){
//        testNotchStatus=0;//STOP
//        PlotPosition();
//        ChangeDirection();
//    }else if(mode == NAVIGATION){
//        Reached_Node();
//        PointCarDirectionToNextNode();
//    }
//
//}

//int Interrupt_3(){ //every wheel spin wheel encoder
//
//
//}



/*
  Dijkstra Algo - Array Fill
*/
void array_fill(int * array, int len, int val) {
    int i;
    for (i = 1; i < len; i++) {
        array[i] = val;
    }
}

/*
  Dijkstra Algo - Logic
*/
int* dijkstra(int graph[][pathpointnumber],int n,int start, int end, int dist[]) {
    int* path=(int*)malloc(sizeof(int)*n);
    int* shortest=(int*)malloc(sizeof(int)*n);
    int* mark=(int*)malloc(sizeof(int)*n);
    int min,v,i,j;
  static int vertexarray[MAXPATHPOINTS];
  static int store[2];
    array_fill(mark,n, 0);
    array_fill(dist,n, inf);

    for(i=0;i<n;i++) {
        dist[i]=graph[start][i];
        if(i!=start&&dist[i]<inf)path[i]=start;
        else path[i]=-1;
    }
    mark[start]=1;
    while(1) {
        min=inf;v=-1;
        // find smallest dist
        for(i=0;i<n;i++) {
            if(!mark[i]) {
                if(dist[i]<min) {min=dist[i];v=i;}
            }
        }
        if(v==-1)break; // if there are no more shortest path
        // update shortest path
        mark[v]=1;
        for(i=0;i<n;i++) {
            if(!mark[i]&&
                    graph[v][i]!=inf&&
                    dist[v]+graph[v][i]<dist[i]) {
                dist[i]=dist[v]+graph[v][i];
                path[i]=v;
            }
        }
    }

  // generate path
  printf("start\t\tend\t\tcost\t\tpath \n");
    for(i=0;i<2;i++) {
    i = end;
        if(i==start) continue;
        array_fill(shortest,n, 0);
        printf("%d\t\t\t",start);
        printf("%d\t\t\t",i);
        printf("%d\t\t\t",dist[i]);
        int k=0;
        shortest[k]=i;
        while(path[shortest[k]]!=start) {
            k++;shortest[k]=path[shortest[k-1]];
        }
        k++;shortest[k]=start;
        for(j=k;j>0;j--) {
            printf("%d->",shortest[j]);
      vertexarray[j] = shortest[j];
      vertexsize++;
        }
        printf("%d\n",shortest[0]);
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
void run_dijkstra() {

  struct Point arr[pathpointnumber];
  int i;
  for (i = 0; i < pathpointnumber; i++) {

    arr[i].x = pointarray[i].x;
    arr[i].y = pointarray[i].y;

  }

  // temp graph
  int G[pathpointnumber][pathpointnumber];

  for (i=0;i<pathpointnumber;i++) {
    printf("%d: %d %d\n", i, arr[i].x, arr[i].y);
    int j;
    for (j=0;j<pathpointnumber;j++) {
      // If it is own node, set cost to 0
      if (arr[j].x == arr[i].x && arr[j].y == arr[i].y) {
        G[i][j] = 0;
      }
      // Check against the x-axis
      else if (arr[j].x == arr[i].x) {
        // Get the cost by subtracting the y-axis
        int cost = arr[j].y - arr[i].y;
        // If cost becomes negative, we abs it
        if (cost < 0) {
          cost = abs(cost);
        }
        G[i][j] = cost;
      }
      // Check against the y-axis
      else if (arr[j].y == arr[i].y) {
        // Get the cost by subtracting the x-axis
        int cost = arr[j].x - arr[i].x;
        // If cost becomes negative, we abs it
        if (cost < 0) {
          cost = abs(cost);
        }
        G[i][j] = cost;
      }
      // Nodes that are not possible to connect will be inf.
      else {
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

  getRoute = dijkstra(G,pathpointnumber,start,end,dist);

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

  for(i=getRoute[1]-1;i>=0;i--) {
        printf("%d", getRoutePath[i]);
  }


  return 0;

}