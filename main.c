#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <sensors/VL53L0X/VL53L0X.h>
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <pi_regulator.h>

#include <movement.h>
#include <width_detection.h>
#include <arm_math.h>


//uncomment to send the FFTs results from the real microphones
//#define SEND_FROM_MIC

//uncomment to use double buffering to send the FFT to the computer
#define DOUBLE_BUFFERING

/*void SendUint8ToComputer(uint8_t* data, uint16_t size)
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}
*/

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}



int main(void)
{

    halInit();
    chSysInit();
    mpu_init();


   //starts the serial communication
    serial_start();
    //starts the USB communication
    usb_start();
    dcmi_start();
    po8030_start();
    //inits the motors
    motors_init();
   // VL53L0X_start();
    start_ThdMovement();
    VL53L0X_start();
    pi_regulator_start();
    process_image_start();
    /* Infinite loop. */
    while (1) {
    	chThdSleepMilliseconds(100);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
