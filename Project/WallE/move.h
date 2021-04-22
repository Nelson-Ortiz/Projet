#ifndef MOVE_H
#define MOVE_H

#define PROX_SENS 8 // 8 sensors au total sensors 0,1,2,...,7
#define LIM_PROX 100
#define DIST_THRESHOLD 100

//Proximity sensors
#define IR1      0
#define IR2     1
#define IR3        2
#define IR4       3
#define IR5       4 
#define IR6         5 
#define IR7     6 
#define IR8      7

void init_movedirections(void);

#endif