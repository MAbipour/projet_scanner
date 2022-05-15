//Code basé sur le TP4 du cours "Systèmes embarqués et robotique" du professeur Francesco Mondada
//modifié par Misha Abipour

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
#include <width_detection.h>


static uint8_t mode_regulator=MODE_REGULATOR_OFF;

//simple PI regulator implementation
int16_t pi_regulator(uint16_t distance, u_int16_t goal){

	int16_t error = 0;
	int16_t speed = 0;
	static int16_t sum_error = 0;

	error = distance - goal;

	//disables the PI regulator if the error is to small
	//this avoids to always move
	if(error > MAX_ERROR_THRESHOLD) {
		mode_regulator=MODE_REGULATOR_OFF;
		stop_motor();
		set_mode_mot(MODE_MOT_ROTATION);
		return(0);
	}


	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}

	else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error + KI * sum_error;

    return speed;
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;

    while(1){
    	time = chVTGetSystemTime();

        if(mode_regulator==MODE_REGULATOR_OFF){
        }

        if(mode_regulator==MODE_REGULATOR_ON){
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

