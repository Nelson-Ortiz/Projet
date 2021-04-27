#include "main.h"
#include "motor.h"
#include "motors.h"

#define DEFAULT_DIR 0 
#define SPIRAL_DIR 1 
#define ANGLE_DIR 2





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

#define PI 3.14159f
#define K 45.0f*PI*2.0f/26.0f // constant for the transformation of  instruction to steps . Depends on the wheels's distance and wheel perimeter


static uint8_t instruction = 0;
static int32_t steps2do [NUM_MOTORS];
static uint8_t direction = DEFAULT_DIR ; 

void angle2steps(uint8_t ins){
    //first we compute the number of steps a motor should do

    float steps_ = K * ins;

    //the motor speed's sign changes in fucntion of the angle. When the speed is negative the counter decreases and viceversa with a postive speed
    //in order to take this into account the number of steps' sign should change as weel
    if (ins<=LIMIT_RIGHT_QUADRANT)
    {
        steps2do[LEFT]=(int32_t) -steps_;
        steps2do[RIGHT]=(int32_t) steps_;
    }
    else
    {
        steps2do[LEFT]=(int32_t) steps_;
        steps2do[RIGHT]=(int32_t) -steps_;
    }
}


static THD_WORKING_AREA(waMotorControl, 1024);
static THD_FUNCTION(MotorControl, arg) {
	(void) arg;
    chRegSetThreadName(__FUNCTION__);
    

    while(1){

    	//first we read the instruction that are given

    	if (instruction == DEFAULT_INS )
    	{
    		
    		left_motor_set_speed(HIGH_SPEED);
            right_motor_set_speed(HIGH_SPEED);


    		direction = DEFAULT_DIR;
    	}
    	
    	else if (instruction >= MIN_ANGLE && instruction <= MAX_ANGLE)
    	{
    		
    		
    		
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

                //we translate the number given into an angle and then into motor steps 
                angle2steps(instruction);
    			//if we are not turning from a previous iteration we start the turning process 
    			left_motor_set_pos(steps2do[LEFT]);
    			right_motor_set_pos(steps2do[RIGHT]);

    			if (instruction<=LIMIT_RIGHT_QUADRANT)
    			{
    				left_motor_set_speed(LOW_SPEED);
    				right_motor_set_speed(-LOW_SPEED);
    			}
    			else
    			{
    				left_motor_set_speed(-LOW_SPEED);
    				right_motor_set_speed(LOW_SPEED);
    			}
                direction=ANGLE_DIR;
    		}
    	}
    	else if (instruction==SPIRAL_INS)
    	{
    		left_motor_set_speed(LOW_SPEED);
    		right_motor_set_speed(HIGH_SPEED);
    		direction=SPIRAL_DIR;
    	}
        else if(instruction==STOP_INS){
            left_motor_set_speed(0);
            right_motor_set_speed(0);
            
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

















































