/*
 * movement.c
 *
 *  Created on: 28 avr. 2022
 *      Author: misha
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <motors.h>
#include <color_detection.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <chprintf.h>
#include <pi_regulator.h>
#include <main.h>
#include <movement.h>
#include <leds.h>


#define SPEED_MOT 100
#define SPEED_MOT_ROT 100
#define SPEED_MOT_HIGH 900 
#define NB_STEP_ROT 7
#define NB_STEP_ROT_PRECI 2
#define NB_STEP_ROT_LONG 168 //correspond à quelques degrés
#define MM_CONV_STEPS 5.7
#define MAX_DIST 1000
#define NB_STEPS_PRECI_ADJUST 60
#define NB_STEPS_CAMERA_ADJUST 2





//misha

static uint8_t mode_mot=MODE_MOT_ROTATION;


void start_motor_straight(int16_t motor_speed, int dist){
	static systime_t time;
    left_motor_set_speed(motor_speed);
    right_motor_set_speed(motor_speed);
    //volatile int i=abs(MM_CONV_STEPS*1000*dist/motor_speed);
    time = chVTGetSystemTime();
    chprintf((BaseSequentialStream *)&SD3, "time=%-7d\r\n", time);
    chThdSleepMilliseconds(abs(MM_CONV_STEPS*1000*dist/motor_speed));
    time = chVTGetSystemTime();
    chprintf((BaseSequentialStream *)&SD3, "time=%-7d\r\n", time);
}



void start_motor_rot(int16_t nb_steps, u_int16_t speed_mot){

	if(nb_steps!=0){

		if(nb_steps>0){
			left_motor_set_speed(-speed_mot);
			right_motor_set_speed(speed_mot);
		}

		if(nb_steps<0){
			left_motor_set_speed(speed_mot);
			right_motor_set_speed(-speed_mot);
		}
		chThdSleepMilliseconds(abs(1000*nb_steps/speed_mot));
	}
}

void start_motor_rot_long(void){
  	left_motor_set_speed(-SPEED_MOT_ROT);
   	right_motor_set_speed(SPEED_MOT_ROT);
   	chThdSleepMilliseconds(1000*NB_STEP_ROT_LONG/SPEED_MOT);

}

void stop_motor(void){
   	left_motor_set_speed(MOTOR_STOP);
   	right_motor_set_speed(MOTOR_STOP);
}


static THD_WORKING_AREA(waThdMovement, 128);
static THD_FUNCTION(ThdMovement, arg) {

	//static uint8_t pos_tab_comp=0;
	//static uint16_t comp_dist[5] = { 0 }
    static uint16_t distance=0;

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){
        switch(mode_mot)
        {
        	case MODE_MOT_ROTATION:
        		set_front_led(FRONT_LED_ON);
        		start_motor_rot(NB_STEP_ROT,SPEED_MOT_ROT);
        		stop_motor();
        		distance=VL53L0X_get_dist_mm();

        	    if(distance<MIN_DIST_DETECTION){
        	    	if(distance<40){
        	    		break;
        	    	}
        	    	start_motor_rot(-1,SPEED_MOT_ROT);
        	    	static uint16_t shrtst_dist=MAX_DIST;
        	    	static int16_t nb_steps_back=0;
        	    	for(uint8_t i=0;i<NB_STEPS_PRECI_ADJUST;i++) {
        	    		if(shrtst_dist>distance){
        	    			shrtst_dist=distance;
        	    			nb_steps_back=i-NB_STEPS_PRECI_ADJUST-NB_STEPS_CAMERA_ADJUST;	//nous réduisons de quelques pas
        	    		}																	//afin que ce soit la camera qui soit
        	    		chThdSleepMilliseconds(100);										//en face de l'objet et pas le capteur de dist
        	    		start_motor_rot(NB_STEP_ROT_PRECI, SPEED_MOT_ROT);
	    		        stop_motor();
        	    		distance=VL53L0X_get_dist_mm();
        	    	}
        	    	start_motor_rot(nb_steps_back*NB_STEP_ROT_PRECI,SPEED_MOT_ROT);
        	    	stop_motor();
        	    	shrtst_dist=MAX_DIST;
        	    	nb_steps_back=0;
        	    	mode_mot=MODE_MOT_STRAIGHT_LINE;

        	    }
        		chThdSleepMilliseconds(100);


        		break;

        	case MODE_MOT_STRAIGHT_LINE:
        		set_front_led(FRONT_LED_OFF);
        		mode_mot=MODE_MOT_OFF;
        		activate_pi_regulator();
        		chThdSleepMilliseconds(100);
        		break;


        	case MODE_MOT_OFF:
        		chThdSleepMilliseconds(100);
        		break;

        }
    }
}

void start_ThdMovement(void){

	chThdCreateStatic(waThdMovement, sizeof(waThdMovement), NORMALPRIO, ThdMovement, NULL);
}

void set_mode_mot(uint8_t mode_mot_param){
	mode_mot=mode_mot_param;
}


