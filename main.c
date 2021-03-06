/*
 * main.c
 *
 *  Created on: avril 2022
 *      Author: Abipour Misha, Leriche Maxime
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ch.h>
#include <hal.h>
#include <memory_protection.h>
#include <motors.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <camera/po8030.h>
#include <main.h>
#include <movement.h>
#include <pi_regulator.h>
#include <width_detection.h>


int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    dcmi_start();
    po8030_start();
    motors_init();
    VL53L0X_start();

    start_ThdMovement();
    pi_regulator_start();
    process_image_start();

    while(1) 
    {
    	chThdSleepMilliseconds(100);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
