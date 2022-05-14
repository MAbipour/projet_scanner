#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

//CONSTANTS MAINLY USED IN "mouvement.c"
#define MODE_MOT_ROTATION 0
#define MODE_MOT_STRAIGHT_LINE 1
#define MODE_MOT_OFF 2

#define MOTOR_STOP 0
#define SPEED_MOT 100
#define SPEED_MOT_ROT 100
#define NB_STEP_ROT 7
#define NB_STEP_ROT_PRECI 2
#define NB_STEP_ROT_LONG 168
#define NB_STEPS_PRECI_ADJUST 60
#define NB_STEPS_CAMERA_ADJUST 2
#define ONE_STEP_BACK -1

#define MAX_DIST_DETECTION 350 //in mm
#define MIN_DIST_DETECTION 40
#define NOT_DEFINED 65535

#define FRONT_LED_OFF 0
#define FRONT_LED_ON 1


//CONSTANTS MAINLY USED IN "pi_regulator.c"
#define MODE_REGULATOR_OFF 0
#define MODE_REGULATOR_ON 1
#define MAX_DISTANCE 			250
#define MIN_ERROR_THRESHOLD		3	//[mm] because of the noise of the camera
#define MAX_ERROR_THRESHOLD		1.5*MAX_DIST_DETECTION
//we added a safety factor of 1.5 in case there is some noise in the TOF sensor

#define KP						5
#define KI 						0.2	//must not be zero
#define MAX_SUM_ERROR 			80
#define GOAL_DISTANCE 45	//distance en mm


//CONSTANTS MAINLY USED IN "width_detection.c"
#define MODE_CAMERA_OFF 0
#define MODE_CAMERA_ON 1

#define X_IMAGE_SIZE 640
#define Y_IMAGE_SIZE 480
#define THRESH_LINE_WIDTH_1 160
#define THRESH_LINE_WIDTH_2 250
#define WIDTH_SLOPE 5
#define MIN_LINE_WIDTH 20
#define NO_LINE_FOUND 65535
#define TOGGLE_BODY_LED 2
#define TOGGLE_FRONT_LED 2



/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
