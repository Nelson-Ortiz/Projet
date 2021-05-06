#include "main.h"
#include "camera.h"
#include "camera/dcmi_camera.h"

#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

// #include <main.h>
#include <camera/po8030.h>



#define GREEN_MSB_PIXEL_MASK 0b00000111
#define GREEN_LSB_PIXEL_MASK 0b11100000
#define TAILLE_PIXEL 2 /*en byte  [ 1 byte en mode FORMAT_YYYY
                                    2 bytes en mode FORMAT_RBG565 ] */
#define AVERAGE_NBR_IMAGE 5
#define BLACK_PIXEL_VALUE 10 

#define DISTANCE(px) ((0.0013f * px *px) - (0.4531f * px) + 47.465f) // polynomial fitting curve ; distance in cm
#define MIN_LIN_VALUE 100 // in order to filter the noise 

#define OFFSET 320 // we put the zero in the middle of the image 

//#define PIX2CM 1

// obstacle status
static uint16_t width = 0;
static uint8_t distance_cm = 0;
static int16_t obstacle_status=0;

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    //Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
    po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
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

    while(1){
        //waits until an image has been captured
        chBSemWait(&image_ready_sem);
        //gets the pointer to the array filled with the last image in RGB565    
        img_buff_ptr = dcmi_get_last_image_ptr();
        // average is done over 
        if(im_ready_counter == 0){
            //difference finie
            // for (int i = 0; i < IMAGE_BUFFER_SIZE-1; i++){
            //     image[i]= (image[i+1]-image[i])/5;
            // }
            SendUint8ToComputer(&image[0], IMAGE_BUFFER_SIZE);
            compute_width(image, IMAGE_BUFFER_SIZE);
            im_ready_counter = AVERAGE_NBR_IMAGE;
            for (int i = 0; i < IMAGE_BUFFER_SIZE; i++){
                image[i]= get_green_pixel(img_buff_ptr+i*TAILLE_PIXEL);
            }
            //chprintf((BaseSequentialStream *)&SD3, "obstacle = %d \n", obstacle_status);
        }
        else{
            im_ready_counter--;
            for (int i = 0; i < IMAGE_BUFFER_SIZE; i++){
                image[i]= (image[i] + get_green_pixel(img_buff_ptr+i*TAILLE_PIXEL))/2;
            }
        } 
    }
}


void init_th_camera(void){
    chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
    chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}


// Other functions

uint8_t get_green_pixel(uint8_t *img_pixel_ptr){
    uint8_t pixel_MSB = *img_pixel_ptr;
    uint8_t pixel_LSB = *(img_pixel_ptr + 1);
    uint8_t green_pixel = 0;
    pixel_MSB = (pixel_MSB & GREEN_MSB_PIXEL_MASK ) <<3;
    pixel_LSB = (pixel_LSB & GREEN_LSB_PIXEL_MASK) >> 5;
    green_pixel = pixel_MSB | pixel_LSB; 
    return green_pixel; 
}

// uint8_t get_ave_pixel(uint8_t *img_pixel_ptr){
//     uint8_t pixel_MSB = *img_pixel_ptr;
//     uint8_t pixel_LSB = *(img_pixel_ptr + 1);
//     uint8_t green_pixel = 0;
//     uint8_t blue_pixel = 0;
//     uint8_t red_pixel = 0;
//     blue_pixel = ()
//     pixel_MSB = (pixel_MSB & GREEN_MSB_PIXEL_MASK ) <<3;
//     pixel_LSB = (pixel_MSB & GREEN_LSB_PIXEL_MASK) >> 5;
//     green_pixel = pixel_MSB | pixel_LSB; 
//     return ave_pixel; 
// }

uint8_t  get_Y_pixel(uint8_t *img_pixel_ptr){ //checker que c'est bien ca ? 
    return *img_pixel_ptr;
}

void compute_width(const uint8_t *image_array, uint16_t line_size){

    uint16_t line_width=0; 
    uint16_t line_start = 0;
    uint16_t pixel_counter=0; 
    uint16_t start_point =0;
    uint16_t pixel_value = 0;
    uint16_t counting = 0; 

    for (int i = 0; i < line_size; ++i)
    {
        pixel_value = *(image_array+i);
        if (pixel_value <= BLACK_PIXEL_VALUE && i<=(line_size-1))
        {
            if (counting == FALSE){
                start_point = i; 
                counting =TRUE;
                pixel_counter++;
            }
            else{ 
                pixel_counter++; 
            }
        }
        else{
            counting = FALSE; 
            if (pixel_counter >= line_width && pixel_counter>=MIN_LIN_VALUE){
                line_width =pixel_counter;
                line_start = start_point;
                pixel_counter=0;
            }
            else { pixel_counter=0; }
        }
    }
    if (line_width== 0){
        obstacle_status=TARGET_NOT_FOUND;
    }
    else{
        obstacle_status= (line_width/2) + line_start - OFFSET; 
    }
    width = line_width;
    distance_cm = DISTANCE(line_width);
}

int16_t get_obstacle_situation(void){
    return obstacle_status;
}

uint8_t get_distance_cm(void){
    return distance_cm;
}







































































