#ifndef SENSOR_TRANSMISSION_H
#define SENSOR_TRANSMISSION_H

#include "common_microros_libs.h"

void timer_callback(rcl_timer_t *timer, int64_t last_call_time);
void sensor_transmission(rclc_support_t *support, rclc_executor_t *executor);

#endif