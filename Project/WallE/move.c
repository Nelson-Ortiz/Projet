#include "main.h"
#include "move.h"

static THD_WORKING_AREA(waMoveDirections, 1024);
static THD_FUNCTION(MoveDirections, arg) {
	(void) arg;
    chRegSetThreadName(__FUNCTION__);
    while(1){



    	
    }
}

void init_movedirections(void){
     chThdCreateStatic(waMoveDirections, sizeof(waMoveDirections), NORMALPRIO, MoveDirections, NULL);
}