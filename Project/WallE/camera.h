#ifndef CAMERA_H
#define CAMERA_H

//constants for the differents parts of the thread
#define IMAGE_BUFFER_SIZE		640  // number of pixels for 1 line
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			70


#define TARGET_NOT_FOUND 400 // this number is used as an error code by pure convention 
 
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
 * @brief   compute the with in pixels of the obstacle seen from the camera
 * 
 * @param *image_array          pointer to the vector where all the pixel's green values are stored
 * @param line_size       		size of the image line. See line_size
 * @param  *black_line          pointer to the vector containing the width and position in pixels of the obstacle 
 *
 */
void get_width(const uint8_t *image_array, uint16_t line_size, uint16_t *black_line );

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


