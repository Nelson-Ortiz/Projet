#include "main.h"
#include "motor.h"
#include "motors.h"

#define DEFAULT 0 // can be spiral or stop
#define FORWARD 1
#define BACKWARD 2 
#define LEFT 3
#define RIGHT 4 
#define STOP 5
#define SPIRAL 6 
#define NUM_DIRECTION 7 


static uint8_t direction[NUM_DIRECTION]; 


static THD_WORKING_AREA(waMotorControl, 1024);
static THD_FUNCTION(MotorControl, arg) {
	(void) arg;
    chRegSetThreadName(__FUNCTION__);
    while(1){



    	
    }
}

void void init_motors(void){
	




    chThdCreateStatic(waMotorControl, sizeof(waMotorControl), NORMALPRIO, MotorControl, NULL);
}