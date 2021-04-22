#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "chprintf.h"
#include "hal.h"
#include "shell.h"

#include "aseba_vm/aseba_node.h"
#include "aseba_vm/skel_user.h"
#include "aseba_vm/aseba_can_interface.h"
#include "aseba_vm/aseba_bridge.h"
#include "audio/audio_thread.h"
#include "audio/play_melody.h"
#include "audio/play_sound_file.h"
#include "audio/microphone.h"
#include "camera/po8030.h"
#include "epuck1x/Asercom.h"
#include "epuck1x/Asercom2.h"
#include "epuck1x/a_d/advance_ad_scan/e_acc.h"
#include "sensors/battery_level.h"
#include "sensors/imu.h"
#include "sensors/mpu9250.h"
#include "sensors/proximity.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "cmd.h"
#include "config_flash_storage.h"
#include "exti.h"
#include "i2c_bus.h"
#include "ir_remote.h"
#include "leds.h"
#include <main.h>
#include "memory_protection.h"
#include "motors.h"
#include "sdio.h"
#include "selector.h"
#include "spi_comm.h"
#include "usbcfg.h"
#include "communication.h"
#include "uc_usage.h"

#include "move.h"
#include "obstacle.h"

#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


static void serial_start(void)
{
    static SerialConfig ser_cfg = {
        115200,
        0,
        0,
        0,
    };

    sdStart(&SD3, &ser_cfg); // UART3.
}




int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    serial_start();
    //starts the USB communication
    usb_start();

    //starts and calibrates the proximity sensors
    proximity_start();
    calibrate_ir();


    //init the message bus of proximity sensors
    messagebus_init(&bus, &bus_lock, &bus_condvar);
    //init thread movedirections
    init_movedirections();


    //init_obstacledetection();


    //wait 2 sec to be sure the e-puck is in a stable position
    chThdSleepMilliseconds(2000);
    set_body_led(LED_ON);
    /* Infinite loop. */
    while (1) {


        chThdSleepMilliseconds(100);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}



// move.c ==================================================================================================
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
