#include "main.h"
#include "long_range.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "sensors/VL53L0X/Api/core/inc/vl53l0x_def.h"
#include "leds.h"
#include "chprintf.h"


#define DIST_THRESHOLD 100
#define ON  1
#define OFF 0




static THD_WORKING_AREA(waLRSensorAnalysis, 1024);
static THD_FUNCTION(LRSensorAnalysis, arg) {
	(void) arg;
    chRegSetThreadName(__FUNCTION__);
    uint16_t dist_mm;

    while(1){
        dist_mm = VL53L0X_get_dist_mm();
        chprintf((BaseSequentialStream *)&SD3, "dist_mm = %d \n", dist_mm);
        if(dist_mm<DIST_THRESHOLD){
            set_front_led(ON);
        }else{
            set_front_led(OFF);
        }

    	chThdSleepMilliseconds(100);
    }
}



void init_LongRangeSensor(void){
     VL53L0X_start();
     chThdCreateStatic(waLRSensorAnalysis, sizeof(waLRSensorAnalysis), NORMALPRIO, LRSensorAnalysis, NULL);
}