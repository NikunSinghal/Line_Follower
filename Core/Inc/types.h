/*
 * types.h
 *
 *  Created on: Apr 17, 2026
 *      Author: jaish
 */

#ifndef INC_TYPES_H_
#define INC_TYPES_H_

typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float last_error;
    float limit;
    float d_filtered;
    float d_alpha;
} PID_Controller;


typedef struct {
	uint16_t adc_raw;
	uint16_t adc_max;
	uint16_t adc_min;
	int16_t weight;
	int threshold;
	int mapped_value;
	int on;


} Sensor;


typedef struct {
	int number_of_sensors;
	Sensor *array;
	int *weights;
	int base_speed;

} Sensor_Array;


typedef enum {
    NO_JUNCTION    = 0,
    LEFT_JUNCTION  = 1,
    RIGHT_JUNCTION = 2,
    T_JUNCTION     = 3,
    CROSS_JUNCTION = 4
} JunctionType;

typedef enum {
    TURN_PRIORITY_LEFT  = 0,
    TURN_PRIORITY_RIGHT = 1
} TurnPriority;


#endif /* INC_TYPES_H_ */
