#ifndef CAMERA_H
#define CAMERA_H

//constants for the differents parts of the project
#define IMAGE_BUFFER_SIZE		640  // number of pixels for 1 line
 
 /**
 * @brief   initialize the motor control thread and other parameters
 * 
 *
 */
void init_th_camera(void);




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