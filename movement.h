/*
 * movement.h
 *
 *  Created on: 28 avr. 2022
 *      Author: misha
 */

#ifndef MOVEMENT_H_
#define MOVEMENT_H_

void start_ThdMovement(void);
void start_motor_straight(int16_t motor_speed, int dist);
void set_mode_mot(uint8_t mode_mot_param);
void start_motor_rot(int16_t nb_steps, u_int16_t speed_mot);
void start_motor_rot_long(void);
void stop_motor(void);

#define MODE_MOT_ROTATION 0
#define MODE_MOT_STRAIGHT_LINE 1
#define MODE_MOT_OFF 2
#define MIN_DIST_DETECTION 300 //in mm
#define FRONT_LED_OFF 0
#define FRONT_LED_ON 1

#endif /* MOVEMENT_H_ */
