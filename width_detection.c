#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <width_detection.h>
#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <pi_regulator.h>
#include <movement.h>
#include <leds.h>
#include <main.h>
#include <camera/po8030.h>





static uint8_t cam_mode=MODE_CAMERA_OFF;


static BSEMAPHORE_DECL(image_ready_sem, TRUE);


uint16_t extract_line_width(uint8_t *buffer){

	uint16_t i = 0, begin = 0, end = 0, width = 0;
	uint8_t stop = 0, line_not_found = 0;
	uint32_t mean = 0;



	//performs an average
	for(uint16_t i = 0 ; i < Y_IMAGE_SIZE ; i++){
		mean += buffer[i];
	}
	mean /= Y_IMAGE_SIZE;

	do{
		//search for a begin
		while(stop == 0 && i < (Y_IMAGE_SIZE - WIDTH_SLOPE))
		{
			//the slope must at least be WIDTH_SLOPE wide and is compared
		    //to the mean of the image
		    if(buffer[i] > mean && buffer[i+WIDTH_SLOPE] < mean)
		    {
		        begin = i;
		        stop = 1;
		    }
		    i++;
		}
		//if a begin was found, search for an end
		if (i < (Y_IMAGE_SIZE - WIDTH_SLOPE) && begin)
		{
		    stop = 0;

		    while(stop == 0 && i < Y_IMAGE_SIZE)
		    {
		        if(buffer[i] > mean && buffer[i-WIDTH_SLOPE] < mean)
		        {
		            end = i;
		            stop = 1;
		        }
		        i++;
		    }
		    //if an end was not found
		    if (i > Y_IMAGE_SIZE || !end)
		    {
		        line_not_found = 1;
		    }
		}
		else//if no begin was found
		{
		    line_not_found = 1;
		}

		//if a line too small has been detected, indicate that no line was found
		if(!line_not_found && (end-begin) < MIN_LINE_WIDTH){
			i = end;
			begin = 0;
			end = 0;
			stop = 0;
			line_not_found = 1;
		}
	}while(i<Y_IMAGE_SIZE-WIDTH_SLOPE);

	if(line_not_found){
		begin = 0;
		end = 0;
		width = NO_LINE_FOUND;
	}

	else{
		width = (end - begin);
		//chThdSleepMilliseconds(100);
	}

	return width;

}



static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to 480 to IMAGE_BUFFER_SIZE of the column 320 + 321
    po8030_advanced_config(FORMAT_RGB565, X_IMAGE_SIZE/2, 0, 2, Y_IMAGE_SIZE, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){

    	if(cam_mode==MODE_CAMERA_ON){
			//starts a capture
			dcmi_capture_start();
			//waits for the capture to be done
			wait_image_ready();
			//signals an image has been captured
			chBSemSignal(&image_ready_sem);
		}
    	chThdSleepMilliseconds(100);
    }
}




static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[X_IMAGE_SIZE] = {0};
	static uint16_t lineWidth = 0;

    while(1){
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565
		img_buff_ptr = dcmi_get_last_image_ptr();

		//Extracts only the red pixels
		for(uint16_t i = 0 ; i < (2 * Y_IMAGE_SIZE) ; i+=2){
			image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
		}

		//search for a line in the image and gets its width in pixels
		lineWidth = extract_line_width(image);
			if(cam_mode==MODE_CAMERA_ON){

				//thin line detected. Action: shakes the head
				if (MIN_LINE_WIDTH<lineWidth && lineWidth<THRESH_LINE_WIDTH_1){

						start_motor_rot(100, 500);
						start_motor_rot(-200, 500);
						start_motor_rot(200, 500);
						start_motor_rot(-100, 500);
						stop_motor();
						chThdSleepMilliseconds(100);
				}

				//thick line detected. Action: blinks the body led
				if (lineWidth>THRESH_LINE_WIDTH_1 && lineWidth<THRESH_LINE_WIDTH_2){
					for(uint8_t i=0; i<4;i++){
						set_body_led(TOGGLE_BODY_LED);
						chThdSleepMilliseconds(500);
					}
				}

				//no line detected. Action: flashes the front led
				if (lineWidth==NO_LINE_FOUND){
					for(uint8_t i=0; i<10;i++){
						set_front_led(TOGGLE_FRONT_LED);
						chThdSleepMilliseconds(75);
					}
				}

			cam_mode=MODE_CAMERA_OFF;
			set_front_led(FRONT_LED_ON);
    		start_motor_rot_long();		//allows the e-puck to move away of the previously scanned object
    		stop_motor();
    		set_mode_mot(MODE_MOT_ROTATION);
			}
			chThdSleepMilliseconds(100);
		}

}




void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
	cam_mode=MODE_CAMERA_OFF;
}



void activate_camera(void){
	cam_mode=MODE_CAMERA_ON;
}
