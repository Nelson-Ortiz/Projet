#include "main.h"
#include "camera.h"
#include "camera/dcmi_camera.h"



#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>



#define TAILLE_PIXEL 1 /*en byte  [ 1 byte en mode FORMAT_YYYY
                                    2 bytes en mode FORMAT_RBG565 ] */
#define AVERAGE_NBR_IMAGE 5
#define BLACK_PIXEL_VALUE 10 

#define DISTANCE(px) ((0.0013f * px *px) - (0.4531f * px) + 47.465f) // polynomial fitting curve ; distance in cm
#define WIDTH 0
#define LINE_START_PIXEL 1 //no used
#define MIN_LIN_VALUE 100 // in order to filter the noise 


#define OFFSET 320 // we put the zero in the midel of the image 


#define PIX2CM 1


static int16_t obstacle_status=0;
static uint16_t line_position = IMAGE_BUFFER_SIZE/2;    //middle

//internal functions
uint8_t  get_pixel(uint8_t *img_pixel_ptr);
void update_obstacle_status( uint16_t width);

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);



/*
 *  Returns the line's width extracted from the image buffer given
 *  Returns 0 if line not found
 */
uint16_t extract_line_width(uint8_t *buffer){
/*===================================================================
* The next algortihm is taken from the TP4 correction of the course 
* "Systèmes embarqués et robotique" gyben by Prof. Francesco Mondada and Dr. Frank Bonnet at EPFL.
* 
* Date: 16/05/2021
====================================================================*/

    uint16_t i = 0, begin = 0, end = 0, width = 0;
    uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
    uint32_t mean = 0;

    //performs an average
    for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
        mean += buffer[i];
    }
    mean /= IMAGE_BUFFER_SIZE;

    do{
        wrong_line = 0;
        //search for a begin
        while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
        { 
            //the slope must at least be WIDTH_SLOPE wide and is compared
            //to the mean of the image
            if(buffer[i] > mean && buffer[i+WIDTH_SLOPE] < mean)
            {
                begin = i;
                stop = 1;
            }
            i++;
        }
        //if a begin was found, search for an end
        if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin)
        {
            stop = 0;
            
            while(stop == 0 && i < IMAGE_BUFFER_SIZE)
            {
                if(buffer[i] > mean && buffer[i-WIDTH_SLOPE] < mean)
                {
                    end = i;
                    stop = 1;
                }
                i++;
            }
            //if an end was not found
            if (i > IMAGE_BUFFER_SIZE || !end)
            {
                line_not_found = 1;
            }
        }
        else//if no begin was found
        {
            line_not_found = 1;
        }

        //if a line too small has been detected, continues the search
        if(!line_not_found && (end-begin) < MIN_LINE_WIDTH){
            i = end;
            begin = 0;
            end = 0;
            stop = 0;
            wrong_line = 1;
        }
    }while(wrong_line);

    if(line_not_found){
        begin = 0;
        end = 0;
        width = 0;
    }else{
        width = (end - begin);
        line_position = (begin + end)/2; //gives the line position.
    }

        return width;
}


static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    //Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
    po8030_advanced_config(FORMAT_YYYY, 0, 300, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
    
    //The next lines can be edited by the user in order to improve the camera recognition in function of the light conditions 
    /*=================================================================*/
    //set contrast to maximum
    /*po8030_set_contrast(127);*/

    // disable auto exposure if very good light conditioning is possible, otherswise let it enabled
    po8030_set_ae(0);
    /*=================================================================*/

    dcmi_enable_double_buffering();
    dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
    dcmi_prepare();

    while(1){
        //starts a capture 
        dcmi_capture_start();
        //waits for the capture to be done
        wait_image_ready();
        
        //signals an image has been captured
        chBSemSignal(&image_ready_sem);     
    }
}


static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    uint8_t *img_buff_ptr;
    uint8_t image[IMAGE_BUFFER_SIZE] = {0};
    uint8_t im_ready_counter = 0;
    uint16_t line_width=0;


    while(1){
        //waits until an image has been captured
        chBSemWait(&image_ready_sem);
        //gets the pointer to the array filled with the last image in RGB565    
        img_buff_ptr = dcmi_get_last_image_ptr();
        
        // average is done over AVERAGE_NBR_IMAGE
        if(im_ready_counter == 0){
            //SendUint8ToComputer(&image[0], IMAGE_BUFFER_SIZE);
            //search for a line in the image and gets its width in pixels
            line_width = extract_line_width(image);


            update_obstacle_status(line_width);
            
            //we restart the averaging
            im_ready_counter = AVERAGE_NBR_IMAGE;
            
            //we store the new values 
            for (int i = 0; i < IMAGE_BUFFER_SIZE; i++){
                image[i]= get_pixel(img_buff_ptr+i*TAILLE_PIXEL);
            }
            
        }

        else{

            im_ready_counter--;

            //the average is done here
            for (int i = 0; i < IMAGE_BUFFER_SIZE; i++){
                image[i]= (image[i] + get_pixel(img_buff_ptr+i*TAILLE_PIXEL))/2;
            }
        }
        
        
    }
}


void init_th_camera(void){
    chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
    chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}


// Other functions

//this function facilitates the switch between each camera modes
uint8_t  get_pixel(uint8_t *img_pixel_ptr){ 
    return *img_pixel_ptr;
} 



void update_obstacle_status( uint16_t width){
    if (width== 0){
        obstacle_status=TARGET_NOT_FOUND;
    }
    else{
        
        obstacle_status= line_position; 
    }

}

int16_t get_obstacle_situation(void){
    return obstacle_status;
}







































































