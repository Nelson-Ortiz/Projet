#include "main.h"
#include "motor.h"
#include "motors.h"

#define DEFAULT_DIR 0 
#define SPIRAL_DIR 1 
#define ANGLE_DIR 2

#define DEFAULT_INS 0 // the robot goes fordward 
#define SPIRAL_INS 121 



#define MAX_ANGLE 120 
#define MIN_ANGLE 1 
#define LIMIT_RIGHT_QUADRANT 60
#define LIMIT_LEFT_QUADRANT 120


#define NUM_MOTORS 2
#define RIGHT 0
#define LEFT 1 

//for the moment the speeds available are defined and constant trough the project
#define HIGH_SPEED 825 // 75% of the max speed 1100 steps/s
#define LOW_SPEED 275 // 25% of the max speed 

static uint8_t instruction = 0;
static int32_t steps2do [NUM_MOTORS];
static uint8_t direction = DEFAULT_DIR ; 


static THD_WORKING_AREA(waMotorControl, 1024);
static THD_FUNCTION(MotorControl, arg) {
	(void) arg;
    chRegSetThreadName(__FUNCTION__);
    

    while(1){

    	//first we read the instruction that are given

    	if (instruction == DEFAULT_INS )
    	{
    		
    		//go fordward

    		direction = DEFAULT_DIR;
    	}
    	
    	else if (instruction >= MIN_ANGLE && instruction <= MAX_ANGLE)
    	{
    		
    		//we translate the number given into an angle and then into motor steps 
    		angle2steps(instruction);
    		
    		//we check if we are  still turning from a previous iteration or if we already finished 
    		if (direction==ANGLE_DIR)
    		{
    			//if we are already turning from a previous iteration we check the status 
    			
    			if (right_motor_get_pos() == 0 && left_motor_get_pos()==0  ) 
    			{
    				//if we already finished turning we put the motors in the default status for the next iteration
    				instruction=DEFAULT_INS;
    			}
    			else
    			{
    				//if we haven't finished turning we keep the status for cheking on the next itertion
    				direction=ANGLE_DIR;
    			}
    		}

    		else
    		{
    			//if we are not turning from a previous iteration we start the turning process 
    			left_motor_set_pos(steps2do[LEFT]);
    			right_motor_set_pos(steps2do[RIGHT]);

    			if (instruction<=LIMIT_LEFT_QUADRANT)
    			{
    				left_motor_set_speed(LOW_SPEED);
    				right_motor_set_speed(-LOW_SPEED);
    			}
    			else
    			{
    				left_motor_set_speed(-LOW_SPEED);
    				right_motor_set_speed(LOW_SPEED);
    			}
    		}
    	}
    	else if (instruction=SPIRAL_INS)
    	{
    		left_motor_set_speed(LOW_SPEED);
    		right_motor_set_speed(HIGH_SPEED);
    		direction=SPIRAL_DIR;
    	}
		else
		{
			left_motor_set_speed(0);
    		right_motor_set_speed(0);
		}
    	
    }
}


void set_direction_motors(uint8_t direction2follow){
	
	instruction = direction2follow; 

}

void inti_th_motor(void){
     chThdCreateStatic(waMotorControl, sizeof(waMotorControl), NORMALPRIO, MotorControl, NULL);
}








































































/*
#define DEFAULT 0 // can be spiral or stop
#define FORWARD 1
#define BACKWARD 2 
#define LEFT 3
#define RIGHT 4 
#define SPIRAL 5 
#define STOP 6
#define NUM_DIRECTION 7 
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
        right_motor_set_speed(550);
        left_motor_set_speed(-550);
    //wait 900ms
    chThdSleepMilliseconds(900);
        //then go fordward
        right_motor_set_speed(-LOW_SPEED);
        left_motor_set_speed(-LOW_SPEED);

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
*/