#ifndef SENSOR_TRANSMISSION_H
#define SENSOR_TRANSMISSION_H

#include "common_microros_libs.h"


#define LED_PIN 25

void timer_callback(rcl_timer_t *timer, int64_t last_call_time);
rcl_ret_t sensor_transmission();

#endif