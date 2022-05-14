#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <motors.h>
#include <width_detection.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <chprintf.h>
#include <pi_regulator.h>
#include <main.h>
#include <movement.h>
#include <leds.h>


static uint8_t mode_mot=MODE_MOT_ROTATION;


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

//This function allows to shift the e-puck's field of view away from the object after it analysed an image
//it prevents the e-puck from analysing the same object twice
void start_motor_rot_avoidance(void){
  	left_motor_set_speed(-SPEED_MOT_ROT);
   	right_motor_set_speed(SPEED_MOT_ROT);
   	chThdSleepMilliseconds(1000*NB_STEP_ROT_AVOIDANCE/SPEED_MOT);
}

void stop_motor(void){
   	left_motor_set_speed(MOTOR_STOP);
   	right_motor_set_speed(MOTOR_STOP);
}


static THD_WORKING_AREA(waThdMovement, 128);
static THD_FUNCTION(ThdMovement, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    uint16_t distance=0;
    while(1){
        switch(mode_mot)
        {
        	case MODE_MOT_ROTATION:
        		set_front_led(FRONT_LED_ON);
        		start_motor_rot(NB_STEP_ROT,SPEED_MOT_ROT);
        		stop_motor();
        		distance=VL53L0X_get_dist_mm();

        		//Algorithm allowing to detect the center of the object we want to analyse
        	    if(distance<MAX_DIST_DETECTION){
        	    	if(distance<MIN_DIST_DETECTION){
        	    		break;
        	    	}
        	    	start_motor_rot(ONE_STEP_BACK,SPEED_MOT_ROT);
        	    	uint8_t pos_smallest_dist=0;
        	    	uint16_t shrtst_dist=NOT_DEFINED;
        	    	int16_t nb_steps_back=0;
        	    	for(uint8_t i=0;i<NB_STEPS_PRECI_ADJUST;i++) {
        	    		if(shrtst_dist>distance){
        	    			shrtst_dist=distance;
        	    			pos_smallest_dist=i;
        	    		}
        	    		start_motor_rot(NB_STEP_ROT_PRECI, SPEED_MOT_ROT);
	    		        stop_motor();
        	    		distance=VL53L0X_get_dist_mm();
        	    	}

        	    	//NB_STEPS_CAMERA_ADJUST allows to adjust the e-puck's position
        	    	//such that the camera is in front of the object and not the distance sensor
        	    	nb_steps_back=pos_smallest_dist-NB_STEPS_PRECI_ADJUST-NB_STEPS_CAMERA_ADJUST;
        	    	start_motor_rot(nb_steps_back*NB_STEP_ROT_PRECI,SPEED_MOT_ROT);
        	    	stop_motor();
        	    	mode_mot=MODE_MOT_STRAIGHT_LINE;
        	    }
        		break;

        	case MODE_MOT_STRAIGHT_LINE:
        		set_front_led(FRONT_LED_OFF);
        		mode_mot=MODE_MOT_OFF;
        		activate_pi_regulator();
        		break;


        	case MODE_MOT_OFF:
        		break;

        }
        chThdSleepMilliseconds(100);
    }
}

void start_ThdMovement(void){
	chThdCreateStatic(waThdMovement, sizeof(waThdMovement), NORMALPRIO, ThdMovement, NULL);
}

void set_mode_mot(uint8_t mode_mot_param){
	mode_mot=mode_mot_param;
}


