#ifndef SENSOR_TRANSMISSION_H
#define SENSOR_TRANSMISSION_H

#include "common_microros_libs.h"


#define LED_PIN 25

extern rcl_publisher_t winch_angle_publisher;
extern rcl_publisher_t rudder_angle_publisher;
extern rcl_publisher_t magnetometer_angle_publisher;

extern std_msgs__msg__Int32 winch_angle;
extern std_msgs__msg__Int32 rudder_angle;
extern std_msgs__msg__Int32 magnetometer_angle;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time);
rcl_ret_t sensor_transmission();

#endif