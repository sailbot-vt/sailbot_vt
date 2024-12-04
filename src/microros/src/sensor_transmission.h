#ifndef SENSOR_TRANSMISSION_H
#define SENSOR_TRANSMISSION_H

#include "common_microros_libs.h"

/**
 * @brief Initializes the sensor transmission node, publisher, and timer.
 * 
 * @param allocator The allocator to be used for memory allocation.
 * @param support   The ROS 2 support structure.
 * @param executor  The ROS 2 executor responsible for executing tasks.
 */
void sensor_transmission_init(rcl_allocator_t *allocator, rclc_support_t *support, rclc_executor_t *executor);


void sensor_transmission_timer_callback(rcl_timer_t *timer, int64_t last_call_time);

/**
 * @brief Cleans up resources associated with the sensor transmission node.
 */
void sensor_transmission_cleanup();

#endif