#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>    //starts the serial communication

#include <chprintf.h>
#include <sensors/VL53L0X/VL53L0X.h>

#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <movement.h>
#include <color_detection.h>


#define MODE_REGULATOR_OFF 0
#define MODE_REGULATOR_ON 1
#define MAX_DISTANCE 			250
#define MIN_ERROR_THRESHOLD		3	//[mm] because of the noise of the camera
#define MAX_ERROR_THRESHOLD		3*MIN_DIST_DETECTION //we added a safety factor of 3
#define KP						5
#define KI 						0.2	//must not be zero
#define MAX_SUM_ERROR 			80


static uint8_t mode_regulator=MODE_REGULATOR_OFF;

//simple PI regulator implementation
int16_t pi_regulator(uint16_t distance, u_int16_t goal){

	int16_t error = 0;
	volatile int16_t speed = 0;

	static int16_t sum_error = 0;

	error = distance - goal;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and
	//the camera is a bit noisy

	if((error > 0) & (error > MAX_ERROR_THRESHOLD)) {
		mode_regulator=MODE_REGULATOR_OFF;			//permet au robot d'éloigner le capteur de distance de l'objet
		stop_motor();
		set_mode_mot(MODE_MOT_ROTATION);
		chThdSleepMilliseconds(100);
		return(0);
	}


	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error + KI * sum_error;

    return (int16_t)speed;
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;
    int16_t speed_correction = 0;

    while(1){
        time = chVTGetSystemTime();

        if(mode_regulator==MODE_REGULATOR_OFF){
        	chThdSleepMilliseconds(100);
        }

        if(mode_regulator==MODE_REGULATOR_ON){
        	//computes the speed to give to the motors
        	//distance_cm is modified by the image processing thread
        	static uint16_t distance=0;
        	distance = VL53L0X_get_dist_mm();
        	speed = pi_regulator(distance, GOAL_DISTANCE);
        	right_motor_set_speed(speed);
        	left_motor_set_speed(speed);
        	if(distance==GOAL_DISTANCE) {
        		right_motor_set_speed(MOTOR_STOP);
        		left_motor_set_speed(MOTOR_STOP);
        		mode_regulator=MODE_REGULATOR_OFF;
        		activate_camera();
        		chThdSleepMilliseconds(100);

        	}
        }
        	//100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
	mode_regulator=MODE_REGULATOR_OFF;
}

void activate_pi_regulator(void){
	mode_regulator=MODE_REGULATOR_ON;
}

