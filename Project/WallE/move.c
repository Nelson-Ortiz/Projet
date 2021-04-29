#include <stdio.h>
#include <stdlib.h>
#include "sensors/proximity.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "communication.h"
#include "leds.h"

#include "main.h"

#include "move.h"
#include "motor.h"
#include "camera.h"
#include "motors.h"

#define PROX_SENS 7 // sensors 0,1,2,...,7
#define LIM_PROX 100

//Proximity sensors
#define IR1     0
#define IR2     1
#define IR3     2
#define IR4     3
#define IR5     4 
#define IR6     5 
#define IR7     6 
#define IR8     7


#define MIN_CAMERA_RANGE 150 //closer than this and the camera stops detecting accurately
#define MAX_CAMERA_RANGE 300 // further than this and the obstacle is too far 

//for the moment the speeds available are defined and constant trough the project
#define HIGH_SPEED 825 // 75% of the max speed 1100 steps/s
#define LOW_SPEED 385 // 35% of the max speed 


//Algos de choix de chemin
void testfunction_stop(proximity_msg_t *prox_values);
void eviter_obstacle(proximity_msg_t *prox_values);
void simple_control(proximity_msg_t *prox_values);
void less_simple_control(proximity_msg_t *prox_values);

//sensors checks
uint8_t check_camera(void);
uint8_t object_detection(void);

static uint8_t following=FALSE;



static THD_WORKING_AREA(waMoveDirections, 1024);
static THD_FUNCTION(MoveDirections, arg) {
	(void) arg;
    chRegSetThreadName(__FUNCTION__);

    /** Inits the Inter Process Communication bus. */
    messagebus_topic_t *proximity_topic = messagebus_find_topic_blocking(&bus, "/proximity");
    proximity_msg_t prox_values;//attention à l'ordre de déclaration avec proximity_start


    //wait 2 sec to be sure the e-puck is in a stable position
    chThdSleepMilliseconds(2000);
    while(1){
        messagebus_topic_wait(proximity_topic, &prox_values, sizeof(prox_values));
        //print("==boot==");


        if (following==TRUE)
        {
            //do following algorithm
            
            if (rand()< RAND_MAX/2)
                    {
                        //turn left as long as the middle sensor doesn't detect something in the camera range
                        following=FALSE;
                        set_body_led(0);
                    }
                    else
                    {
                        //turn right as long as the middle sensor doesn't detect something in the camera range

                        following=TRUE;
                        set_body_led(1);
                    } 
        }                   
        else{

            if (object_detection()==FALSE)
            {
                //fordward avoiding obstacles 
                less_simple_control(&prox_values);
                following=FALSE;
            }
            else
            {   
                //if we detected an object in the camera working proximity we check its nature
                if (check_camera()==FALSE)
                {
                    //if it is an obstacle we avoid it 

                    //randomly chose left or rigth 
                    if (rand()< RAND_MAX/2)
                    {
                        //turn left as long as the middle sensor doesn't detect something in the camera range
                        left_motor_set_speed(LOW_SPEED);
                        right_motor_set_speed(HIGH_SPEED);
                        following=FALSE;
                    }
                    else
                    {
                        //turn right as long as the middle sensor doesn't detect something in the camera range
                        left_motor_set_speed(HIGH_SPEED);
                        right_motor_set_speed(LOW_SPEED);
                        following=FALSE;
                    }
                }
                //if we detect the target we start the "target chase " algorithm
                else
                {
                    following=TRUE;
                }
            }
        }
        //we let the other sensors do their measures
        chThdSleepMilliseconds(1000);
    }
}

void init_movedirections(void){
     chThdCreateStatic(waMoveDirections, sizeof(waMoveDirections), NORMALPRIO, MoveDirections, NULL);
}


void testfunction_stop(proximity_msg_t *prox_values){
    int dist=0;
    for (int i = 0; i < PROX_SENS; ++i)
        {
            //chprintf((BaseSequentialStream *)&SD3, "value3%4d, \r\n", prox_values.delta[3]);
            if (prox_values->delta[i]>LIM_PROX)
            {
                dist++;
            }
        }
        if (dist>0)
        {
            set_body_led(1);
            set_direction_motors(STOP_INS);
            dist=0;
        }
        else{
            set_body_led(0);
            set_direction_motors(SPIRAL_INS);
            dist=0;
        }
}

//algos de control 

#define FORWARD    set_direction_motors(DEFAULT_INS);
#define LEFT        set_direction_motors(70); // quasi demi-tour vers la gauche
#define RIGHT       set_direction_motors(50); //mettre un angle vers la droite ?


void simple_control(proximity_msg_t *prox_values){
    if (prox_values->delta[IR7]<LIM_PROX && prox_values->delta[IR8]<LIM_PROX){
        if (prox_values->delta[IR1]<LIM_PROX && prox_values->delta[IR1]<LIM_PROX){
            FORWARD
        }else{
            LEFT
        }
    }else{
        RIGHT
    }
}

void less_simple_control(proximity_msg_t *prox_values){
    if (prox_values->delta[IR8]<LIM_PROX){
        if (prox_values->delta[IR1]<LIM_PROX){
            if (prox_values->delta[IR7]<LIM_PROX){
                if (prox_values->delta[IR2]<LIM_PROX){
                    FORWARD
                }else{
                   LEFT
                }
            }else{
                RIGHT
            }
        }else if(prox_values->delta[IR7]<LIM_PROX){
            LEFT
        }else{
            RIGHT
        }
    }else if(prox_values->delta[IR1]<LIM_PROX){
        if (prox_values->delta[IR2]<LIM_PROX){
            RIGHT
        }else{
            LEFT
        }
    }else if (prox_values->delta[IR2]<LIM_PROX){
        RIGHT
    }else{
        LEFT
    }
}

void eviter_obstacle(proximity_msg_t *prox_values){
        if (prox_values->delta[IR1]<LIM_PROX){
            if(prox_values->delta[IR8]<LIM_PROX){
                //droit
                set_direction_motors(DEFAULT_INS);
            }
            else if(prox_values->delta[IR2]<LIM_PROX){
                //droite
                set_direction_motors(30); //mettre un angle de 45° vers la droite ?
            }
            else{
                //demi-tour
                set_direction_motors(60);
            }
        }
        else if(prox_values->delta[IR8]<LIM_PROX){
            if(prox_values->delta[IR7]<LIM_PROX){
                //gauche
                set_direction_motors(90); // angle de 45° vers la gauche ?
            }
            else{
                //demi-tour
                set_direction_motors(60);
            }
        }else{
            //demi-tour
            set_direction_motors(60);
        }
}

uint8_t object_detection(void){
    //chekc the long range sensor to see if an object entered the camera working range
    if (VL53L0X_get_dist_mm()<=MIN_CAMERA_RANGE && VL53L0X_get_dist_mm()>=MAX_CAMERA_RANGE)
    {
        //out of range or too close
        return FALSE; 
    }
    else
    {
        //an object entered the camera detection range 
        return TRUE;
    }

}

uint8_t check_camera(void){
    //checks the camera to see if the obstacle is a target or an obstacle 
    if (get_obstacle_situation()==TARGET_NOT_FOUND)
    {
        return FALSE;
    }
    else
    {
        return TRUE;
    }

}
