#include "sensors/proximity.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "communication.h"
#include "leds.h"

#include "main.h"

#include "move.h"

#define PROX_SENS 7 // sensors 0,1,2,...,7
#define LIM_PROX 100

static THD_WORKING_AREA(waMoveDirections, 1024);
static THD_FUNCTION(MoveDirections, arg) {
	(void) arg;
    chRegSetThreadName(__FUNCTION__);

    /** Inits the Inter Process Communication bus. */
    messagebus_topic_t *proximity_topic = messagebus_find_topic_blocking(&bus, "/proximity");
    proximity_msg_t prox_values;//attention à l'ordre de déclaration avec proximity_start

    
    int dist=0;

    //wait 2 sec to be sure the e-puck is in a stable position
    chThdSleepMilliseconds(2000);
    while(1){
          	messagebus_topic_wait(proximity_topic, &prox_values, sizeof(prox_values));
        //print("==boot==");
    


        for (int i = 0; i < PROX_SENS; ++i)
        {
            //chprintf((BaseSequentialStream *)&SD3, "value3%4d, \r\n", prox_values.delta[3]);
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

        
        chThdSleepMilliseconds(500);
    }
}

void init_movedirections(void){
     chThdCreateStatic(waMoveDirections, sizeof(waMoveDirections), NORMALPRIO, MoveDirections, NULL);
}
