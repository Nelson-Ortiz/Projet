#include <stdio.h>
#include <stdlib.h>
#include "sensors/proximity.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include "communication.h"
#include "leds.h"
#include "audio/play_melody.h"

#include "main.h"

#include "move.h"
#include "camera.h"
#include "motors.h"


/* The next parameters can vary strongly in fucntion of the luminosity. 
*  They should can tunned in function  of the environnement in wich the robot is tested.*/

/*=====================================================================================*/
#define MIN_CAMERA_RANGE 200 //further than this and the camera signal becomes not fiable
#define LIM_PROX 150 // If the sensor value is higher than this then it means an obstacle was detected
/*=====================================================================================*/

//for the moment the speeds available are defined and constant trough the project
#define HIGH_SPEED 935 // 85% of the max speed 1100 steps/s
#define LOW_SPEED 605 // 55% of the max speed 

 //the speed values are a constant and depends in the geometry of the robot and the steps motors 
#define SPEED_15_DEG 140
#define SPEED_45_DEG 405
#define SPEED_90_DEG 660
#define SPEED_150_DEG 1095

#define NO_PROX_DETECTED 10 //arbitrary value that allows us to recognise if no sensor proximity was detected

#define TARGET_DETECTED 0
#define SEARCH 1 
#define NEAR_OBJECT 2


#define PROX_SENS 7 // sensors 0,1,2,...,7
//Proximity sensors
#define IR1     0
#define IR2     1
#define IR3     2
#define IR4     3
#define IR5     4 
#define IR6     5 
#define IR7     6 
#define IR8     7


//mouvement algorithms
void search_algorithm(void); // it is done when no object is detected 
void near_object_algorithm(void); // when a near object is detected it points the camera towards it 

//does a little show with the leds 
void lighting_garland(void);

//sensors checks
uint8_t check_camera(void);
uint8_t object_detection(void);
void check_prox(proximity_msg_t *prox_values);

// Indicates to the main switch wich algorithm to execute
static uint8_t status=SEARCH;

//helps with the algorithms keeping track of the number of iterations of the thread. Approximately 1 iteration =0.5 sec
static uint8_t loop_counter=0; 

//If a near object was detected, it indicates the sensor's number that detected it. 
//if none was detected it has a default value called NO_PROX_DETECTED
static uint8_t active_prox_sensor= NO_PROX_DETECTED; 



static THD_WORKING_AREA(waMoveDirections, 128);
static THD_FUNCTION(MoveDirections, arg) {
	(void) arg;
    chRegSetThreadName(__FUNCTION__);

    /** Inits the Inter Process Communication bus. */
    messagebus_topic_t *proximity_topic = messagebus_find_topic_blocking(&bus, "/proximity");
    proximity_msg_t prox_values;//attention ?? l'ordre de d??claration avec proximity_start


    //wait 2 sec to be sure the e-puck is in a stable position
    chThdSleepMilliseconds(2000);

    //playMelody(IMPOSSIBLE_MISSION,ML_SIMPLE_PLAY,NULL); 
    playNote(140, 1000); 

    while(1){

        messagebus_topic_wait(proximity_topic, &prox_values, sizeof(prox_values));

        /*First we check if a proximity object was detected
            if not -> continue with SEARCH algorithm
            if yes -> switch to NEAR_OBJECT algorithm
            if we already found the target then we ignore the proximity sensors

        */
        check_prox(&prox_values); 
    

        switch(status){

            case TARGET_DETECTED:
                //first we stop the robot
                left_motor_set_speed(0);
                right_motor_set_speed(0);   

                //it it doesn't sees the target anymore it comes back to a SEARCH mouvement  
                if (check_camera()==FALSE){
                    loop_counter=0;
                    set_front_led(LED_OFF);
                    clear_leds();
                    status=SEARCH;
                }
                
                //does a little show with the lights
                lighting_garland();
                loop_counter++;
                
                //once we've recognised the target long enough (???2 sec), we play a sound 
                if (loop_counter==4){
                    playNote(280, 1000);
                }

                //we stop the counting in order to avoid an overflow
                if (loop_counter>4)
                {
                    loop_counter=5;
                }
                break;

            case SEARCH:
                clear_leds();
                search_algorithm(); 
                break;

            case NEAR_OBJECT:
                set_front_led(LED_OFF);
                clear_leds();
                near_object_algorithm();
                break;
        }        
                
        
        //we let the other sensors do their measures
        chThdSleepMilliseconds(500);
    }
}

void init_movedirections(void){
     chThdCreateStatic(waMoveDirections, sizeof(waMoveDirections), NORMALPRIO, MoveDirections, NULL);
}

void search_algorithm(void){

    //if we don't detect an object we simply go fordward
    if (object_detection()==FALSE){
        left_motor_set_speed(HIGH_SPEED);
        right_motor_set_speed(HIGH_SPEED);          
    }

    else{

        //once an object is detected we wait 3 iterations (???1.5 sec) in order to have a better image
        /*====================================*/
        if (loop_counter==3){
            loop_counter=0;
        }

        else if (loop_counter<3 && loop_counter>0){
            left_motor_set_speed(0);
            right_motor_set_speed(0);
            loop_counter++;
            return;
        }
        /*====================================*/
        else if (loop_counter==0){
            //when it detects an obstacle it stops for a moment in order to stabilize the camera signal
            left_motor_set_speed(0);
            right_motor_set_speed(0);
            loop_counter++;
            return;
        }
        //if we detected an object in the camera working proximity we check its nature
        if (check_camera()==FALSE){
            //if it is an obstacle we avoid it 
            set_front_led(LED_OFF);
            //Randomly choose left or rigth 
            if (rand()< RAND_MAX/2){
                //turn left as long as the middle sensor doesn't detect something in the camera range
                left_motor_set_speed(-LOW_SPEED-100);
                right_motor_set_speed(HIGH_SPEED+100);
            }
            else{
                //turn right as long as the middle sensor doesn't detect something in the camera range
                left_motor_set_speed(HIGH_SPEED+100);
                right_motor_set_speed(-LOW_SPEED-100);
            }
        }
        //if we detect the target we start the "target chase " algorithm
        else{
            set_front_led(LED_ON);
            loop_counter=0;
            status=TARGET_DETECTED;
             
        }
    }

}

/* Once a near object was detected this algorithm executes a number of instructions that will point the robot towards the object
    * This is done in function of the times we entered the thread, or iterations:
    *
    *       iteration=0 -> It detects the object and it turns in the direction of the proximity sensor that detected it 
    *       iteration=1 -> It stops turning and it moves backwards    
    *       iteration 2 to 5 -> It stops and waits to the camera to stabilize its signal (???1.5 sec)
    *       iteration 5 -> It switchs to the SEARCH algorithm so it can chect the nature of the object (obstacle or target)
*/
void near_object_algorithm (void) {

    //once we finished turning the robot so it faces the object detected by the corresponding proximity sensor
    // the robot moves back so it can have a better obstacle's view
    if (loop_counter==1){
        left_motor_set_speed(-HIGH_SPEED);
        right_motor_set_speed(-HIGH_SPEED);
        loop_counter++;
        return;
    }

    // after it moved backwards it stops for the duration of 3 iterations (???1,5 seconds) so the camera signal stabilizes itself
    else if (loop_counter>=2 && loop_counter<5){
        left_motor_set_speed(0);
        right_motor_set_speed(0);
        loop_counter++;
        return;
    }

    //finally it switches back to the SEARCHING algorithm so it can check the nature of the object (obstacle or target)
    else if (loop_counter>=5){
        loop_counter=0;
        status=SEARCH;
        return;
    }

    //in function of the sensor that dectected the object we turn of the corresponding angle
    switch(active_prox_sensor){
        case IR1:
        //first we turn
        if (loop_counter==0){
            loop_counter++; 
            left_motor_set_speed(SPEED_15_DEG);
            right_motor_set_speed(-SPEED_15_DEG);
        }
        break;
        case IR2:
            //first we turn
            if (loop_counter==0){
            loop_counter++;                 
            left_motor_set_speed(SPEED_45_DEG);
            right_motor_set_speed(-SPEED_45_DEG);
            }
            break;
        case IR3:
            //first we turn
            if (loop_counter==0){
                loop_counter++;
                left_motor_set_speed(SPEED_90_DEG);
                right_motor_set_speed(-SPEED_90_DEG);
            }
            break;
        case IR4:
            //first we turn
            if (loop_counter==0){
                loop_counter++;
                left_motor_set_speed(SPEED_150_DEG);
                right_motor_set_speed(-SPEED_150_DEG);
            }
            break;
        case IR5:
            //first we turn
            if (loop_counter==0){
                loop_counter++;
                left_motor_set_speed(-SPEED_150_DEG);
                right_motor_set_speed(SPEED_150_DEG);
            }
            break;
        case IR6:
            //first we turn
            if (loop_counter==0){
                loop_counter++;
                left_motor_set_speed(-SPEED_90_DEG);
                right_motor_set_speed(SPEED_90_DEG);
            }
            break;
        case IR7:
            //first we turn
            if (loop_counter==0){
                loop_counter++;
                left_motor_set_speed(-SPEED_45_DEG);
                right_motor_set_speed(SPEED_45_DEG);
            }
            break;
        case IR8:
            //first we turn
            if (loop_counter==0){
                loop_counter++;
                left_motor_set_speed(-SPEED_15_DEG);
                right_motor_set_speed(SPEED_15_DEG);
            }
            break;

        case NO_PROX_DETECTED:
            loop_counter=0;
            status=SEARCH;
            break;    
    }
    return;
}




uint8_t object_detection(void){
    //chekc the long range sensor to see if an object entered the camera working range
    if (VL53L0X_get_dist_mm()<=MIN_CAMERA_RANGE){
        //an object entered the camera detection range 
        return TRUE; 
    }
    else{
        //out of range 
        return FALSE;
    }

}

uint8_t check_camera(void){
    //checks the camera to see if the obstacle is a target or an obstacle 
    if (get_obstacle_situation()==TARGET_NOT_FOUND){
        return FALSE;
    }
    else{
        return TRUE;
    }

}

void check_prox(proximity_msg_t *prox_values){
    
    //If we already detected the target or we are cheking an obstacle we just detected we ignore the proximity sensors new readings
    if (status!=SEARCH){
        return;
    }
    /* Otherwise we read the data from the proximity sensors and we check if one of them has a value bigger than the
    *  LIM_PROX, that means that it detected a near object.
    */
    else{
        active_prox_sensor=NO_PROX_DETECTED;
        for (int i = 0; i < PROX_SENS; ++i){
            if (prox_values->delta[i]>LIM_PROX){
            active_prox_sensor=i;
            }
        }
    }
    
    //chprintf((BaseSequentialStream *)&SD3, "sensor = %d \n", sensor);
    //if no close object was detected
    if (active_prox_sensor == NO_PROX_DETECTED){
        set_body_led(LED_ON);
        //if we were on the NEAR_OBJECT algorithm that means that we just finished checking an object, so we switch to SEARCH
        if (status==NEAR_OBJECT){
            loop_counter=0;
            status=SEARCH;
        }
        //if we were SEARCHING  then we do nothing
        else if (status==SEARCH){
            status=SEARCH;
        }
        
        else if (status==TARGET_DETECTED){
            status=TARGET_DETECTED;
        }
    }
    // if we detected a near object 
    else{
        //if we were searching then we switch to NEAR_OBJECT algorithm that will point the camera towards the object
        if (status == SEARCH){
            loop_counter=0;
            status=NEAR_OBJECT;         
        }
        //IF we were on the others alogrithms then we don't switch them
        else if (status==TARGET_DETECTED){
            status=TARGET_DETECTED;
        }
        else{
            status=NEAR_OBJECT;
        }
    }
}

// turns on and off the leds in a circular cycle. 
void lighting_garland(void){
    static uint8_t led_counter = 0;

    if(led_counter == 0){
        led_counter ++;
        set_led(LED1,LED_TOGGLE);
    }
    else if(led_counter == 1){
        led_counter ++;
        set_led(LED3,LED_TOGGLE);
    }
    else if(led_counter == 2){
        led_counter ++;
        set_led(LED5,LED_TOGGLE);
    }
    else {
        led_counter = 0;
        set_led(LED7,LED_TOGGLE);
    }
}
