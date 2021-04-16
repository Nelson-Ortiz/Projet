#include "main.h"
#include "motor.h"
#include "motors.h"

#define DEFAULT 0 // can be spiral or stop
#define FORWARD 1
#define BACKWARD 2 
#define LEFT 3
#define RIGHT 4 
#define SPIRAL 5 
#define STOP 6
#define NUM_DIRECTION 7 

//for the moment the speeds available are defined and constant trough the project
#define HIGH_SPEED 825 // 75% of the max speed 1100 steps/s
#define LOW_SPEED 275 // 25% of the max speed 


//static uint8_t direction[NUM_DIRECTION]; 

/*
static THD_WORKING_AREA(waMotorControl, 1024);
static THD_FUNCTION(MotorControl, arg) {
	(void) arg;
    chRegSetThreadName(__FUNCTION__);
    while(1){



    	
    }
}*/

void instruction_motor(uint8_t direction){
	switch(direction) {
		
	case DEFAULT: 
		//here the default will be spiral to the left
		right_motor_set_speed(HIGH_SPEED);
		left_motor_set_speed(LOW_SPEED);
		break;
	case FORWARD:
		right_motor_set_speed(LOW_SPEED);
		left_motor_set_speed(LOW_SPEED);
		break;
	case BACKWARD:
		right_motor_set_speed(-LOW_SPEED);
		left_motor_set_speed(-LOW_SPEED);
		break;
	case LEFT:
			
		//counter is in steps. 1000 steps for 1 turn of the wheel
			
		//turn 90deg to the left
		right_motor_set_pos(500);
		left_motor_set_pos(-500);

		//then go fordward
		right_motor_set_speed(LOW_SPEED);
		left_motor_set_speed(LOW_SPEED);

		break;
	case RIGHT:
		//sam as left but inversing the instructions
			
		//turn 90deg to the right
		right_motor_set_pos(-500);
		left_motor_set_pos(500);

		//then go fordward
		right_motor_set_speed(LOW_SPEED);
		left_motor_set_speed(LOW_SPEED);
		break;
	case SPIRAL:
		right_motor_set_speed(HIGH_SPEED);
		left_motor_set_speed(LOW_SPEED);
		break;
	case STOP:
		right_motor_set_speed(0);
		left_motor_set_speed(0);
		break;
	//if we don't have a matching direction the robot will stop
	default:
		right_motor_set_speed(0);
		left_motor_set_speed(0);
	}



}