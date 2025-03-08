#ifndef RUDDER_CONTROL_H
#define RUDDER_CONTROL_H

#include "common_microros_libs.h"

/**
 * @brief Initializes the sensor transmission node, publisher, and timer.
 * 
 * @param allocator The allocator to be used for memory allocation.
 * @param support   The ROS 2 support structure.
 * @param executor  The ROS 2 executor responsible for executing tasks.
*/

void rudder_control_init(rcl_allocator_t *allocator, rclc_support_t *support, rclc_executor_t *executor);

void uros_log(const char *msg, uint8_t severity, const char *file, const char *function);

void rudder_control_loop();

void desired_rudder_angle_received_callback(const void *msg_in);

void should_relay_be_open_callback(const void *msg_in);

void zero_rudder_encoder_callback(const void * request_msg, void * response_msg);

#endif