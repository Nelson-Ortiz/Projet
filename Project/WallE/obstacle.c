#include "main.h"
#include "obstacle.h"

static THD_WORKING_AREA(waObstacleDetection, 1024);
static THD_FUNCTION(ObstacleDetection, arg) {
	(void) arg;
    chRegSetThreadName(__FUNCTION__);
    while(1){



    	
    }
}

void init_obstacledetection(void){
    chThdCreateStatic(waObstacleDetection, sizeof(waObstacleDetection), NORMALPRIO, ObstacleDetection, NULL);
}