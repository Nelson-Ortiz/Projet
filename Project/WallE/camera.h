#ifndef CAMERA_H
#define CAMERA_H

//constants for the differents parts of the thread
#define IMAGE_BUFFER_SIZE		640  // number of pixels for 1 line
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			40
#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			2 
#define PXTOCM					1570.0f //experimental value
#define GOAL_DISTANCE 			10.0f
#define MAX_DISTANCE 			25.0f
#define ERROR_THRESHOLD			0.1f	//[cm] because of the noise of the camera
#define KP						800.0f
#define KI 						3.5f	//must not be zero
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)

#define TARGET_NOT_FOUND 400 // this number is used as an error code as pure convention 
 
 /**
 * @brief   initialize the camera capture and precessing thread
 * 
 *
 */
void init_th_camera(void);
/*
 * @brief   Computes the obstacle distance from the robot 
 *
 * @return        The distance in cm
 */
uint8_t get_distance_cm(void);
/*
 * @brief   filtrates the green pixels from the image 
 * 
 * @param *img_pixel_ptr          pointer to the pixel postion in the buffer where the thread capture_image 
 *								put the values for the green/red/blue chanels
 *
 * @return              The green pixel value
 *
 */
uint8_t get_green_pixel(uint8_t *img_pixel_ptr);
uint8_t  get_Y_pixel(uint8_t *img_pixel_ptr);
/*
 * @brief   compute the with in pixels of the obstacle seen from the camera
 * 
 * @param *image_array          pointer to the vector where all the pixel's green values are stored
 * @param line_size       		size of the image line. See line_size
 * @param  *black_line          pointer to the vector containing the width and position in pixels of the obstacle 
 *
 */
void get_width(const uint8_t *image_array, uint16_t line_size, uint16_t *black_line );
/*
 * @brief   updates the obstacle status if it is detected and/or if moved it's pixel positions
 * 
 */

void update_obstacle_status( uint16_t* properties);
/*
 * @brief   return the obstacle situation
 * 
 *
 * @return              [-320,320] middle of the obstacle position in pixels
 *						400 No obstacle found
 *
 */

int16_t get_obstacle_situation(void);




#endif




//template 
 /*
 * @brief   Configures some parameters of the camera
 * 
 * @param fmt           format of the image. See format_t
 * @param imgsize       size of the image. See image_size_t
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end or if wrong imgsize
 *
 */