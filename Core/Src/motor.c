/*
 * motor.c
 *
 *  Created on: Apr 10, 2026
 *      Author: jaish
 */


#include "main.h"
#include "stm32f4xx_hal.h"
#include  "utils.h"
#include "sensor_module.h"
#include "motor.h"

void set_motor_speed(int left_motor, int right_motor, float battery_voltage) {


    if (battery_voltage < 1.0f) {
        battery_voltage = 8.4f;
    }

    left_motor = (int)((left_motor * 8.4f) / battery_voltage);
    right_motor = (int)((right_motor * 8.4f) / battery_voltage);

    left_motor  = constrain_int(left_motor,  -999, 999);
    right_motor = constrain_int(right_motor, -999, 999);

    if (left_motor < 0 && right_motor > 0) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, right_motor);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1000+left_motor);
    }
    else if (left_motor < 0 && right_motor < 0) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000+right_motor);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1000+left_motor);
    }
    else if (left_motor > 0 && right_motor < 0) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000+right_motor);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, left_motor);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1000);
    }
    else if (left_motor >= 0 && right_motor >= 0) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, right_motor);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, left_motor);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1000);
    }
    /*else if (left_motor == 0 && right_motor == 0) {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1000);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1000);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
        }*/
}


void follow_line(int correction , Sensor_Array* sensor_array) {


    int left_speed = sensor_array->base_speed + correction;
    int right_speed = sensor_array->base_speed - correction;



    set_motor_speed(left_speed, right_speed , battery_voltage(dma_buffer));
}


void swing_turn_left(Sensor_Array *sa, int speed) {

	set_motor_speed(speed * 0.6f, speed * 0.6f, battery_voltage(dma_buffer));
	HAL_Delay(60);

	set_motor_speed(-speed * 0.8f, speed * 0.8f, battery_voltage(dma_buffer));
    HAL_Delay(100);


    while (1) {
        Sync_Sensors(sa);
        processSensors(sa);
        binarizeSensors(sa);

        if (sa->array[3].on == 1 || sa->array[4].on == 1) {
            break;
        }

        HAL_Delay(5);
    }


    set_motor_speed(0, 0, battery_voltage(dma_buffer));
    HAL_Delay(30);

    set_motor_speed(speed * 0.4f, speed * 0.4f, battery_voltage(dma_buffer));
    HAL_Delay(40);

    set_motor_speed(0, 0, battery_voltage(dma_buffer));
}

void swing_turn_right(Sensor_Array *sa, int speed) {
	set_motor_speed(speed * 0.6f, speed * 0.6f, battery_voltage(dma_buffer));
	HAL_Delay(60);

	set_motor_speed(speed * 0.8f, -speed * 0.8f, battery_voltage(dma_buffer));
	HAL_Delay(100);

    while (1) {
        Sync_Sensors(sa);
        processSensors(sa);
        binarizeSensors(sa);

        if (sa->array[3].on == 1 || sa->array[4].on == 1) {
            break;
        }

        HAL_Delay(5);
    }


    set_motor_speed(0, 0, battery_voltage(dma_buffer));
    HAL_Delay(30);

    set_motor_speed(speed * 0.4f, speed * 0.4f, battery_voltage(dma_buffer));
    HAL_Delay(40);

    set_motor_speed(0, 0, battery_voltage(dma_buffer));
}



void handle_junction(Sensor_Array *sa, JunctionType j , int speed) {
    switch (j) {

        case LEFT_JUNCTION:
            swing_turn_left(sa, speed);
            break;

        case RIGHT_JUNCTION:
            swing_turn_right(sa, speed);
            break;

        case T_JUNCTION:
        	set_motor_speed(750, 750, battery_voltage(dma_buffer));
        	HAL_Delay(100);
        	set_motor_speed(0, 0, battery_voltage(dma_buffer));
        	HAL_Delay(30);
        	break;

        case CROSS_JUNCTION:
        	HAL_Delay(10);
            break;


        case NO_JUNCTION:
        	break;
        default:
            break;
    }
}








