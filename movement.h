/*
 * movement.h
 *
 *  Created on: 28 avr. 2022
 *      Author: misha
 */

#ifndef MOVEMENT_H_
#define MOVEMENT_H_

void start_ThdMovement(void);
void set_mode_mot(uint8_t mode_mot_param);
void start_motor_rot(int16_t nb_steps, u_int16_t speed_mot);
void start_motor_rot_avoidance(void);
void stop_motor(void);



#endif /* MOVEMENT_H_ */
