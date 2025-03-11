#ifndef RUDDER_CONTROL_H
#define RUDDER_CONTROL_H

#include "common_microros_libs.h"

#define CLOCKWISE 1
#define COUNTER_CLOCKWISE 0

#define MAX_RUDDER_CURRENT 2000
#define MAX_WINCH_CURRENT 2000

#if !BOATMODE
#define RUDDER_GAIN 50
#else
#define RUDDER_GAIN 200
#endif

#define WINCH_GAIN 150

#define WINCH_ZERO_POINT 200

#define MAX_RUDDER_ANGLE 35
#define MIN_RUDDER_ANGLE -35

#define MAX_WINCH_ANGLE 580
#define MIN_WINCH_ANGLE -600

#define MAX_SAIL_ANGLE 90
#define MIN_SAIL_ANGLE 0

#define ACCEPTABLE_RUDDER_ERROR 1
#define ACCEPTABLE_SAIL_ERROR 0.5

#define RUDDER_ANGLE_OFFSET 36
#define WINCH_ANGLE_OFFSET 0

#define MIN_TIME_BETWEEN_MOTOR_STEPS 2000 // microseconds

// This period is the length of the delay between steps, which controls the
// stepper motor's speed.  You can increase the delay to make the stepper motor
// go slower.  If you decrease the delay, the stepper motor will go faster, but
// there is a limit to how fast it can go before it starts missing steps.
#define STEP_PERIOD_US 2000


/**
 * @brief Initializes the microros node, publisher, and timer.
 * 
 * @param allocator The allocator to be used for memory allocation.
 * @param support   The ROS 2 support structure.
 * @param executor  The ROS 2 executor responsible for executing tasks.
*/

void application_init(rcl_allocator_t *allocator, rclc_support_t *support, rclc_executor_t *executor);

void uros_log(const char *msg, uint8_t severity, const char *file, const char *function);

void application_loop();

void desired_rudder_angle_received_callback(const void *msg_in);

void desired_winch_angle_received_callback(const void *msg_in);

void should_relay_be_open_callback(const void *msg_in);

void zero_rudder_encoder_callback(const void *msg_in);

void init_relay_bus();

#endif