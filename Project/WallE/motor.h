#ifndef MOTOR_H
#define MOTOR_H


 /**
 * @brief  Set the robot to go to the direction indicated
 * 
 * @param direction		Default(0); Forward(1); Backward(2);Left(3);
 *						Right(4);Spiral(5); stop(6)
 *						Otherwise it's set to stop
 */

void instruction_motor(uint8_t direction);

#endif