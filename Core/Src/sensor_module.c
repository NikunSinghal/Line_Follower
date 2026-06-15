/*
 * sensor_module.c
 *
 *  Created on: Apr 11, 2026
 *      Author: jaish
 */


#include "stm32f4xx_hal.h"
#include "utils.h"
#include "motor.h"
#include "sensor_module.h"





volatile uint16_t dma_buffer[NUM_SENSORS + 1];




void Initialize_Sensor_Array(Sensor_Array *sensor_array){


	for(int i = 0 ; i < sensor_array->number_of_sensors ; i++){
		sensor_array->array[i].weight = sensor_array->weights[i];
		sensor_array->array[i].adc_min = 0;
		sensor_array->array[i].adc_max = 4095;
		sensor_array->array[i].threshold = SENSOR_THRESHOLD;
	}




}

void Sync_Sensors(Sensor_Array *sensor_array){

	sensor_array->array[0].adc_raw = dma_buffer[3];
	sensor_array->array[1].adc_raw = dma_buffer[4];
	sensor_array->array[2].adc_raw = dma_buffer[2];
	sensor_array->array[3].adc_raw = dma_buffer[5];
	sensor_array->array[4].adc_raw = dma_buffer[1];
	sensor_array->array[5].adc_raw = dma_buffer[6];
	sensor_array->array[6].adc_raw = dma_buffer[0];
	sensor_array->array[7].adc_raw = dma_buffer[7];

//	sensor_array->array[0].adc_raw = dma_buffer[0];
//	sensor_array->array[1].adc_raw = dma_buffer[1];
//	sensor_array->array[2].adc_raw = dma_buffer[2];
//	sensor_array->array[3].adc_raw = dma_buffer[3];
//	sensor_array->array[4].adc_raw = dma_buffer[4];
//	sensor_array->array[5].adc_raw = dma_buffer[5];
//	sensor_array->array[6].adc_raw = dma_buffer[6];

}



void autoCalibrate(Sensor_Array *sensor_array , uint32_t duration_ms, int speed) {

    for (int i = 0; i < sensor_array->number_of_sensors; i++) {
    	sensor_array->array[i].adc_min = 0;
    	sensor_array->array[i].adc_max = 4095;
    }

    set_motor_speed(-speed, speed , battery_voltage(dma_buffer));

    uint32_t startTime = HAL_GetTick();


    while (HAL_GetTick() - startTime < duration_ms) {


        for (int i = 0; i < NUM_SENSORS; i++) {
        	if (sensor_array->array[i].adc_raw < sensor_array->array[i].adc_min) sensor_array->array[i].adc_min = sensor_array->array[i].adc_raw;
        	if (sensor_array->array[i].adc_raw > sensor_array->array[i].adc_max) sensor_array->array[i].adc_max = sensor_array->array[i].adc_raw;

        }


        HAL_Delay(1);
    }


    set_motor_speed(0, 0 , battery_voltage(dma_buffer));
}




void processSensors(Sensor_Array *sensor_array)
{
    for (int i = 0; i < sensor_array->number_of_sensors; i++)
    {
        int range = sensor_array->array[i].adc_max - sensor_array->array[i].adc_min;
        if (range <= 0) range = 1;

        int norm = ((int)(sensor_array->array[i].adc_raw - sensor_array->array[i].adc_min) * 1000) / range;
        norm = constrain_int(norm, 0, 1000);


        sensor_array->array[i].mapped_value = 1000 - norm;
    }

}


void binarizeSensors(Sensor_Array *sensor_array)
{

    for (int i = 0; i < NUM_SENSORS; i++)
    {
    	if (i == 0 || i == 7) {
    	            sensor_array->array[i].on = 0;
    	            continue;
    	        }

        if (sensor_array->array[i].adc_raw > sensor_array->array[i].threshold)
        {
            sensor_array->array[i].on = 1;
        }
        else
        {
        	sensor_array->array[i].on = 0;
        }
    }
}





int get_line_error(Sensor_Array *sensor_array) {
	long adc_sum = 0;
    long weighted_sum = 0;
    for(int i = 0 ; i < NUM_SENSORS ; i++){
    	weighted_sum += (sensor_array->array[i].mapped_value)*(sensor_array->array[i].weight);
    	adc_sum += sensor_array->array[i].mapped_value;
    }

    return ((int)(weighted_sum)/(int)adc_sum);
}

int get_line_error_digital(Sensor_Array* sensor_array) {
    float weighted_sum = 0;
    int active_sensors = 0;
    static int last_error = 0 ;

    for(int i = 0 ; i < NUM_SENSORS ; i++){
        if (sensor_array->array[i].on == 1) {
            weighted_sum += sensor_array->array[i].weight;
            active_sensors++;
        }
    }

    if(active_sensors == 0) return last_error;



    float true_position = weighted_sum / active_sensors;

    int out = (int)(true_position * 50);

    last_error = out;

    return out;
}





int calculate_pid(PID_Controller *pid, int error, float dt) {

	// 1. Quadratic Proportional Term
	float Kp2 = 0.002f; // Tune this: 0.001 to 0.005 range
	float P = (pid->Kp * (float)error) + (Kp2 * (float)error * fabsf((float)error));

    pid->integral += ((float)error * dt);
    if (pid->integral > pid->limit) pid->integral = pid->limit;
    if (pid->integral < -pid->limit) pid->integral = -pid->limit;
    float I = pid->Ki * pid->integral;


    float raw_d = ((float)error - (float)pid->last_error)/ dt;
    pid->d_filtered = pid->d_alpha * pid->d_filtered + (1.0f - pid->d_alpha) * raw_d;  // EMA
    float D = pid->Kd * pid->d_filtered;
    pid->last_error = error;

    float output = P + I + D;

    if (output > pid->limit) return (int)pid->limit;
    if (output < -pid->limit) return (int)-pid->limit;

    return (int)output;
}


int count_active_sensors(Sensor_Array *sensor_array) {
    int active_count = 0;
    for (int i = 0; i < sensor_array->number_of_sensors; i++) {
    	active_count += sensor_array->array[i].on;
    }
    return active_count;
}



JunctionType detect_junction(Sensor_Array *sensor_array) {

    int n = sensor_array->number_of_sensors;
    int outer_width = n / 4;
    if (outer_width < 1) outer_width = 1;

    int left_outer_count  = 0;
    int right_outer_count = 0;
    int center_count      = 0;
    int total             = 0;

    for (int i = 0; i < n; i++) {
        total += sensor_array->array[i].on;

        if (i < outer_width) {
            left_outer_count  += sensor_array->array[i].on;
        } else if (i >= n - outer_width) {
            right_outer_count += sensor_array->array[i].on;
        } else {
            center_count      += sensor_array->array[i].on;
        }
    }


    if (center_count == 0){ return NO_JUNCTION;}

    int left_branch  = (left_outer_count  >= 1);
    int right_branch = (right_outer_count >= 1);

    if (left_branch && right_branch) return T_JUNCTION;
    if (left_branch)                 return LEFT_JUNCTION;
    if (right_branch)                return RIGHT_JUNCTION;

    return NO_JUNCTION;
}




JunctionType detect_junction_digital(Sensor_Array *sensor_array) {
    int sensor_count = count_active_sensors(sensor_array);

	if (sensor_count >= 3) {
		if (sensor_count >= 5 && sensor_array->array[3].on == 1 && sensor_array->array[4].on == 1  ) {
			return T_JUNCTION;
		}

		else if (sensor_array->array[1].on == 1  && (sensor_array->array[3].on == 1 || sensor_array->array[2].on == 1) && sensor_array->array[5].on == 0) {
			return LEFT_JUNCTION;
		}

		else if (sensor_array->array[6].on == 1  && (sensor_array->array[5].on == 1 || sensor_array->array[4].on == 1) && sensor_array->array[2].on == 0)  {
			return RIGHT_JUNCTION;
		}
	}
	return NO_JUNCTION;
}





