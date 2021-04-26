#include "sensors/proximity.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "communication.h"
#include "leds.h"

#include "main.h"

#include "move.h"
#include "motor.h"

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

//Algos de choix de chemin
void testfunction_stop(proximity_msg_t *prox_values);
void eviter_obstacle(proximity_msg_t *prox_values);
void simple_control(proximity_msg_t *prox_values);
void less_simple_control(proximity_msg_t *prox_values);

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
    
        less_simple_control(&prox_values); // comme ça on pourra changer facilement de fonctions :)
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
                clear_leds();
                set_led(LED1,LED_ON);
                set_direction_motors(DEFAULT_INS);
            }
            else if(prox_values->delta[IR2]<LIM_PROX){
                //droite
                clear_leds();
                set_led(LED2,LED_ON);
                set_direction_motors(30); //mettre un angle de 45° vers la droite ?
            }
            else{
                //demi-tour
                clear_leds();
                set_led(LED5,LED_ON);
                set_direction_motors(60);
            }
        }
        else if(prox_values->delta[IR8]<LIM_PROX){
            if(prox_values->delta[IR7]<LIM_PROX){
                //gauche
                clear_leds();
                set_led(LED2,LED_ON);
                set_direction_motors(90); // angle de 45° vers la gauche ?
            }
            else{
                //demi-tour
                clear_leds();
                set_led(LED5,LED_ON);
                set_direction_motors(60);
            }
        }else{
            //demi-tour
            clear_leds();
            set_led(LED5,LED_ON);
            set_direction_motors(60);
        }
}