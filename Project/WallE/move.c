#include "main.h"
#include "move.h"

#include "leds.h"
#include "sensors/proximity.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "communication.h"

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


static THD_WORKING_AREA(waMoveDirections, 1024);
static THD_FUNCTION(MoveDirections, arg) {
	(void) arg;
    chRegSetThreadName(__FUNCTION__);

    /** Inits the Inter Process Communication bus. */
    messagebus_topic_t *proximity_topic = messagebus_find_topic_blocking(&bus, "/proximity");
    proximity_msg_t prox_values;//attention à l'ordre de déclaration avec proximity_start

    
    int dist=0;
    uint16_t dist_mm;

    //wait 2 sec to be sure the e-puck is in a stable position
    chThdSleepMilliseconds(2000);
    while(1){
          	messagebus_topic_wait(proximity_topic, &prox_values, sizeof(prox_values));
        //print("==boot==");
    
        dist_mm = VL53L0X_get_dist_mm(); // y a til une cible devant moi ?
        if(dist_mm<DIST_THRESHOLD){
            set_front_led(LED_ON);
        }
        // FONCTION TST DE NELSON
        for (int i = 0; i < PROX_SENS; ++i)
        {
            if (prox_values.delta[i]>LIM_PROX)
            {
                dist++;
            }
        }
        if (dist>0)
        {
            set_body_led(1);
            dist=0;
        }
        else{
            set_body_led(0);
            dist=0;
        }


        // check des obstacles
        // if(prox_values.delta[IR1]<LIM_PROX){
        //     if(prox_values.delta[IR8]<LIM_PROX){ // pas d'obstacle devant 
        //         // on va tout droit
        //         // clear_leds();
        //         // set_led(LED1,LED_ON);
        //     }
        //     else if(prox_values.delta[IR2]<LIM_PROX && prox_values.delta[IR3]<LIM_PROX){ // pas d'obstacle à 45
        //         //on va a droite 
        //         // clear_leds();
        //         // set_led(LED3,LED_ON);
        //     }
        // }
        // else if(    prox_values.delta[IR8]<LIM_PROX
        //                 && prox_values.delta[IR7]<LIM_PROX 
        //                 && prox_values.delta[IR6]<LIM_PROX){

        //     // on va a gauche
        //     // clear_leds();
        //     // set_led(LED7,LED_ON);
        // } 
        // else if(prox_values.delta[IR4]<LIM_PROX && prox_values.delta[IR5]<LIM_PROX){
        //     // on fait demi-tour
        //     // clear_leds();
        //     // set_led(LED5,LED_ON);
        // }

         
        
        //envoyer la valeur uint8 de commande 
        chThdSleepMilliseconds(500);
    }
}

void init_movedirections(void){
     chThdCreateStatic(waMoveDirections, sizeof(waMoveDirections), NORMALPRIO, MoveDirections, NULL);
}