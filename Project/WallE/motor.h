#ifndef MOTOR_H
#define MOTOR_H


 /**
 * @brief  Set the robot to go to the direction indicated
 * 
 * @param direction		0 : default, fordward in this case
 *						1-120: translated in an anlge for the robot
 *						121 : spiral
 *						122-127: others 
 */
void set_direction_motors(uint8_t direction2follow);

 /**
 * @brief   initialize the motor control thread and other parameters
 * 
 *
 */
void inti_th_motor(void);


#endif




//template 
 /**
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