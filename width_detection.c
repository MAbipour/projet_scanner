///*
// * color_detection.c
// *
// *  Created on: 3 mai 2022
// *      Author: misha
// */
//
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <color_detection.h>
#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <pi_regulator.h>
#include <movement.h>
#include <leds.h>
#include <main.h>
#include <camera/po8030.h>



#define THRESH_RED 10
#define THRESH_BLUE 10
#define THRESH_GREEN 10
#define X_IMAGE_SIZE 640
#define Y_IMAGE_SIZE 480
#define BORDER_EFFECT_CORR 50 //eliminates the drop of intensity on the borders of the captured image
#define THRESH_LINE_WIDTH_1 100
#define THRESH_LINE_WIDTH_2 200
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			50
#define TOGGLE_BODY_LED 2

#define PXTOMM					15700 //experimental value
//#define MAX_DISTANCE 			250
#define OTHER_IMAGE_BUFFER_SIZE 480
#define NO_LINE_FOUND 65535
#define TOGGLE_FRONT_LED 2

#define PXTOCM					1570.0f //experimental value
#define GOAL_DISTANCE1 			45.0f
#define MAX_DISTANCE 			25.0f
#define ERROR_THRESHOLD			0.1f	//[cm] because of the noise of the camera
//
//#define IMAGE_BUFFER_SIZE		640
//#define OTHER_IMAGE_BUFFER_SIZE		480
//#define WIDTH_SLOPE				5
//#define MIN_LINE_WIDTH			40
//#define ROTATION_THRESHOLD		10
//#define ROTATION_COEFF			2
//#define PXTOCM					1570.0f //experimental value
//#define GOAL_DISTANCE 			10.0f
//#define MAX_DISTANCE 			25.0f
//#define ERROR_THRESHOLD			0.1f	//[cm] because of the noise of the camera
//#define KP						800.0f
//#define KI 						3.5f	//must not be zero
//#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)
//

static uint8_t cam_mode=CAMERA_OFF;


static BSEMAPHORE_DECL(image_ready_sem, TRUE);


uint16_t extract_line_width(uint8_t *buffer){

	uint16_t i = 0, begin = 0, end = 0, width = 0;
	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
	uint32_t mean = 0;

	static uint16_t last_width = PXTOCM/GOAL_DISTANCE1;

	//performs an average
	for(uint16_t i = 0 ; i < OTHER_IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
	}
	mean /= OTHER_IMAGE_BUFFER_SIZE;

	do{
		wrong_line = 0;
		//search for a begin
		while(stop == 0 && i < (OTHER_IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
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
		if (i < (OTHER_IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin)
		{
		    stop = 0;

		    while(stop == 0 && i < OTHER_IMAGE_BUFFER_SIZE)
		    {
		        if(buffer[i] > mean && buffer[i-WIDTH_SLOPE] < mean)
		        {
		            end = i;
		            stop = 1;
		        }
		        i++;
		    }
		    //if an end was not found
		    if (i > OTHER_IMAGE_BUFFER_SIZE || !end)
		    {
		        line_not_found = 1;
		    }
		}
		else//if no begin was found
		{
		    line_not_found = 1;
		}

		//if a line too small has been detected, continues the search
		if(!line_not_found && (end-begin) < MIN_LINE_WIDTH){
			i = end;
			chprintf((BaseSequentialStream *)&SD3, "width1=%-7d\r\n", end-begin);
			begin = 0;
			end = 0;
			stop = 0;
			wrong_line = 1;
			line_not_found = 1;
		}
	}while(wrong_line);

	if(line_not_found){
		begin = 0;
		end = 0;
//		width = last_width;
		width = NO_LINE_FOUND;
	}

	else{
		last_width = width = (end - begin);
		chprintf((BaseSequentialStream *)&SD3, "width2=%-7d\r\n", width);
		chThdSleepMilliseconds(500);

		//line_position = (begin + end)/2; //gives the line position.
	}

	//sets a maximum width or returns the measured width
//	if((PXTOCM/width) > MAX_DISTANCE){
//		return PXTOCM/MAX_DISTANCE;
//	}else{
//		return width;
//	}
	return width;

}
//
//
//uint16_t extract_line_width(uint8_t *buffer){
//
//	uint16_t i = 0, begin = 0, end = 0, width = 0;
//	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
//	uint32_t mean = 0;
//
//	static uint16_t last_width = PXTOCM/GOAL_DISTANCE;
//
//	//performs an average
//	for(uint16_t i = 0 ; i < OTHER_IMAGE_BUFFER_SIZE ; i++){
//		mean += buffer[i];
//	}
//	mean /= OTHER_IMAGE_BUFFER_SIZE;
//
//	do{
//		wrong_line = 0;
//		//search for a begin
//		while(stop == 0 && i < (OTHER_IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
//		{
//			//the slope must at least be WIDTH_SLOPE wide and is compared
//		    //to the mean of the image
//		    if(buffer[i] > mean && buffer[i+WIDTH_SLOPE] < mean)
//		    {
//		        begin = i;
//		        stop = 1;
//		    }
//		    i++;
//		}
//		//if a begin was found, search for an end
//		if (i < (OTHER_IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin)
//		{
//		    stop = 0;
//
//		    while(stop == 0 && i < OTHER_IMAGE_BUFFER_SIZE)
//		    {
//		        if(buffer[i] > mean && buffer[i-WIDTH_SLOPE] < mean)
//		        {
//		            end = i;
//		            stop = 1;
//		        }
//		        i++;
//		    }
//		    //if an end was not found
//		    if (i > OTHER_IMAGE_BUFFER_SIZE || !end)
//		    {
//		        line_not_found = 1;
//		    }
//		}
//		else//if no begin was found
//		{
//		    line_not_found = 1;
//		}
//
//		//if a line too small has been detected, continues the search
//		if(!line_not_found && (end-begin) < MIN_LINE_WIDTH){
//			i = end;
//			begin = 0;
//			end = 0;
//			stop = 0;
//			wrong_line = 1;
//		}
//	}while(wrong_line);
//
//	if(line_not_found){
//		begin = 0;
//		end = 0;
//		width = last_width;
//	}else{
//		last_width = width = (end - begin);
//		chThdSleepMilliseconds(500);
//		chprintf((BaseSequentialStream *)&SD3, "width=%-7d\r\n", width);
//		//line_position = (begin + end)/2; //gives the line position.
//	}
//
//	//sets a maximum width or returns the measured width
//	if((PXTOCM/width) > MAX_DISTANCE){
//		return PXTOCM/MAX_DISTANCE;
//	}else{
//		return width;
//	}
//
//}
//

//
//uint16_t extract_line_width(uint8_t *buffer){
//
//	uint16_t i = 0, begin = 0, end = 0, width = 0;
//	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
//	uint32_t mean = 0;
//
//	static uint16_t last_width = PXTOMM/GOAL_DISTANCE;
//
//	//performs an average
//	for(uint16_t i = 0 ; i < Y_IMAGE_SIZE ; i++){
//		mean += buffer[i];
//	}
//	mean /= Y_IMAGE_SIZE;
//
//	do{
//		wrong_line = 0;
//		//search for a begin
//		while(stop == 0 && i < (Y_IMAGE_SIZE - WIDTH_SLOPE))
//		{
//			//the slope must at least be WIDTH_SLOPE wide and is compared
//		    //to the mean of the image
//		    if(buffer[i] > mean && buffer[i+WIDTH_SLOPE] < mean)
//		    {
//		        begin = i;
//		        stop = 1;
//		    }
//		    i++;
//		}
//		//if a begin was found, search for an end
//		if (i < (Y_IMAGE_SIZE - WIDTH_SLOPE) && begin)
//		{
//		    stop = 0;
//
//		    while(stop == 0 && i < Y_IMAGE_SIZE)
//		    {
//		        if(buffer[i] > mean && buffer[i-WIDTH_SLOPE] < mean)
//		        {
//		            end = i;
//		            stop = 1;
//		        }
//		        i++;
//		    }
//		    //if an end was not found
//		    if (i > Y_IMAGE_SIZE || !end)
//		    {
//		        line_not_found = 1;
//		    }
//		}
//		else//if no begin was found
//		{
//		    line_not_found = 1;
//		}
//
//		//if a line too small has been detected, continues the search
//		if(!line_not_found && (end-begin) < MIN_LINE_WIDTH){
//			i = end;
//			begin = 0;
//			end = 0;
//			stop = 0;
//			wrong_line = 1;
//		}
//	}while(wrong_line);
//
//	if(line_not_found){
//		begin = 0;
//		end = 0;
//		width = last_width;
//	}else{
//		last_width = width = (end - begin);
//		//line_position = (begin + end)/2; //gives the line position.
//	}
//
//	//sets a maximum width or returns the measured width
//	if((PXTOMM/width) > MAX_DISTANCE){
//		return PXTOMM/MAX_DISTANCE;
//	}else{
//		return width;
//	}
//}


static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
//	po8030_advanced_config(FORMAT_RGB565, 0, 10, X_IMAGE_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
    po8030_advanced_config(FORMAT_RGB565, X_IMAGE_SIZE/2, 0, 2, 480, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){

    	if(cam_mode==CAMERA_ON){
    		//chBSemWait(&image_ready_sem);
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

	bool send_to_computer = true;

    while(1){
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565
		img_buff_ptr = dcmi_get_last_image_ptr();

		//Extracts only the red pixels
		for(uint16_t i = 0 ; i < (2 * OTHER_IMAGE_BUFFER_SIZE) ; i+=2){
			//extracts first 5bits of the first byte
			//takes nothing from the second byte
			image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
		}

		//search for a line in the image and gets its width in pixels
		lineWidth = extract_line_width(image);
//    		//waits until an image has been captured
//    		chBSemWait(&image_ready_sem);
//    		//gets the pointer to the array filled with the last image in RGB565
//			img_buff_ptr = dcmi_get_last_image_ptr();
//
//			//Extracts only the red pixels
//			for(uint16_t i = 0 ; i < (2 * Y_IMAGE_SIZE) ; i+=2){
//				//image[i/2] = (uint8_t)img_buff_ptr[i+1]&0x1F;
//				image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
//			}
//
	//		lineWidth = extract_line_width(image);
//			if(send_to_computer){
//					//sends to the computer the image
//					SendUint8ToComputer(image, X_IMAGE_SIZE);
//			}
//			send_to_computer = !send_to_computer;

			chprintf((BaseSequentialStream *)&SD3, "width3=%-7d\r\n", lineWidth);
			//if ((lineWidth>60) & (lineWidth<90)){		//d�tection cible avec petite ligne
			if(cam_mode==CAMERA_ON){


				if (MIN_LINE_WIDTH<lineWidth && lineWidth<160){

						start_motor_rot(100, 500);
						start_motor_rot(-200, 500);
						start_motor_rot(200, 500);
						start_motor_rot(-100, 500);
						stop_motor();
						chThdSleepMilliseconds(100);
				}

				//if ((lineWidth>160) & (lineWidth<250)){		//d�tection cible avec grande ligne
				if (lineWidth>160 && lineWidth<250){
					for(uint8_t i=0; i<4;i++){
						set_body_led(TOGGLE_BODY_LED);
						chThdSleepMilliseconds(500);
					}
				}

				if (lineWidth>250 && 500>lineWidth){
					start_motor_rot(-100, 500);
					start_motor_rot(200, 500);
					start_motor_rot(-200, 500);
					start_motor_rot(100, 500);
					stop_motor();
					chThdSleepMilliseconds(100);
				}


				if (lineWidth==NO_LINE_FOUND){
					for(uint8_t i=0; i<10;i++){
						set_front_led(TOGGLE_FRONT_LED);
						chThdSleepMilliseconds(75);
					}
				}

			cam_mode=CAMERA_OFF;
			set_front_led(FRONT_LED_ON);
    		start_motor_rot_long();				//permet au robot d'�loigner le capteur de distance de l'objet
    		stop_motor();
    		set_mode_mot(MODE_MOT_ROTATION);
    		//chBSemSignal(&image_ready_sem);
			}
			chThdSleepMilliseconds(100);
		}

}




void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
	cam_mode=CAMERA_OFF;
}



void activate_camera(void){
	cam_mode=CAMERA_ON;
}
//
//#include "ch.h"
//#include "hal.h"
//#include <chprintf.h>
//#include <usbcfg.h>
//
//#include <main.h>
//#include <camera/po8030.h>
//
//#include <color_detection.h>
//
//
//static float distance_cm = 0;
//static uint16_t line_position = IMAGE_BUFFER_SIZE/2;	//middle
//
////semaphore
//static BSEMAPHORE_DECL(image_ready_sem, TRUE);
//
///*
// *  Returns the line's width extracted from the image buffer given
// *  Returns 0 if line not found
//// */
//uint16_t extract_line_width(uint8_t *buffer){
//
//	uint16_t i = 0, begin = 0, end = 0, width = 0;
//	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
//	uint32_t mean = 0;
//
//	static uint16_t last_width = PXTOCM/GOAL_DISTANCE;
//
//	//performs an average
//	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
//		mean += buffer[i];
//	}
//	mean /= OTHER_IMAGE_BUFFER_SIZE;
//
//	do{
//		wrong_line = 0;
//		//search for a begin
//		while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
//		{
//			//the slope must at least be WIDTH_SLOPE wide and is compared
//		    //to the mean of the image
//		    if(buffer[i] > mean && buffer[i+WIDTH_SLOPE] < mean)
//		    {
//		        begin = i;
//		        stop = 1;
//		    }
//		    i++;
//		}
//		//if a begin was found, search for an end
//		if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin)
//		{
//		    stop = 0;
//
//		    while(stop == 0 && i < IMAGE_BUFFER_SIZE)
//		    {
//		        if(buffer[i] > mean && buffer[i-WIDTH_SLOPE] < mean)
//		        {
//		            end = i;
//		            stop = 1;
//		        }
//		        i++;
//		    }
//		    //if an end was not found
//		    if (i > IMAGE_BUFFER_SIZE || !end)
//		    {
//		        line_not_found = 1;
//		    }
//		}
//		else//if no begin was found
//		{
//		    line_not_found = 1;
//		}
//
//		//if a line too small has been detected, continues the search
//		if(!line_not_found && (end-begin) < MIN_LINE_WIDTH){
//			i = end;
//			begin = 0;
//			end = 0;
//			stop = 0;
//			wrong_line = 1;
//		}
//	}while(wrong_line);
//
//	if(line_not_found){
//		begin = 0;
//		end = 0;
//		width = last_width;
//	}else{
//		last_width = width = (end - begin);
//		line_position = (begin + end)/2; //gives the line position.
//	}
//
//	//sets a maximum width or returns the measured width
//	if((PXTOCM/width) > MAX_DISTANCE){
//		return PXTOCM/MAX_DISTANCE;
//	}else{
//		return width;
//	}
//}
//
//
//
//
//
//
//
//static THD_WORKING_AREA(waCaptureImage, 256);
//static THD_FUNCTION(CaptureImage, arg) {
//
//    chRegSetThreadName(__FUNCTION__);
//    (void)arg;
//
//	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
//	//po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
//    po8030_advanced_config(FORMAT_RGB565, IMAGE_BUFFER_SIZE/2, 0, 2, 480, SUBSAMPLING_X1, SUBSAMPLING_X1);
//    dcmi_enable_double_buffering();
//	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
//	dcmi_prepare();
//
//    while(1){
//        //starts a capture
//		dcmi_capture_start();
//		//waits for the capture to be done
//		wait_image_ready();
//		//signals an image has been captured
//		chBSemSignal(&image_ready_sem);
//    }
//}
//
//
//static THD_WORKING_AREA(waProcessImage, 1024);
//static THD_FUNCTION(ProcessImage, arg) {
//
//    chRegSetThreadName(__FUNCTION__);
//    (void)arg;
//
//	uint8_t *img_buff_ptr;
//	uint8_t image[IMAGE_BUFFER_SIZE] = {0};
//	uint16_t lineWidth = 0;
//
//	bool send_to_computer = true;
//
//    while(1){
//    	//waits until an image has been captured
//        chBSemWait(&image_ready_sem);
//		//gets the pointer to the array filled with the last image in RGB565
//		img_buff_ptr = dcmi_get_last_image_ptr();
//
//		//Extracts only the red pixels
//		for(uint16_t i = 0 ; i < (2 * OTHER_IMAGE_BUFFER_SIZE) ; i+=2){
//			//extracts first 5bits of the first byte
//			//takes nothing from the second byte
//			image[i/2] = (uint8_t)img_buff_ptr[i+1]&0x1F;
//		}
//
//		//search for a line in the image and gets its width in pixels
//		//lineWidth = extract_line_width(image);
//
//		//converts the width into a distance between the robot and the camera
//		//if(lineWidth){
//		//	distance_cm = PXTOCM/lineWidth;
//		//}
//
////		if(send_to_computer){
////			//sends to the computer the image
////			SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
////		}
//		//invert the bool
//		send_to_computer = !send_to_computer;
//    }
//}
//
//float get_distance_cm(void){
//	return distance_cm;
//}
//
//uint16_t get_line_position(void){
//	return line_position;
//}
//
//void process_image_start(void){
//	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
//	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
//}
//