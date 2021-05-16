#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "chprintf.h"
#include "hal.h"
#include "shell.h"

#include "audio/audio_thread.h"
#include "audio/play_melody.h"
#include "audio/play_sound_file.h"
#include "camera/po8030.h"
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
#include "spi_comm.h"
#include "usbcfg.h"
#include "communication.h"
#include "uc_usage.h"

#include "move.h"
#include "camera.h"

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

    /*System initialisation*/
    halInit();
    chSysInit();
    mpu_init();
    //starts the serial communication
    serial_start();
    //starts the ToF sensor
    VL53L0X_start();
    //starts the camera
    dcmi_start();
    po8030_start();
    //starts the motors
    motors_init();
    //starts proximity sensors
    proximity_start();
    //Digital-to-Analog converter start. Needed for the speakers
    dac_start();


    //starts the capture and image processing thread thread 
    init_th_camera();

    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);
    
    //initialize the moving thread
    init_movedirections();

    //important to have this after the bus init
    calibrate_ir();

    
    //stars the speaker thread
    playMelodyStart();

    //wait 2 sec to be sure the e-puck is in a stable position
    chThdSleepMilliseconds(2000);

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



