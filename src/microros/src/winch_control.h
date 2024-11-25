#ifndef WINCH_CONTROL_H
#define WINCH_CONTROL_H

#include "common_microros_libs.h"

void winch_control(rcl_allocator_t *allocator, rclc_support_t *support, rclc_executor_t *executor);
void winch_control_timer_callback(rcl_timer_t *timer, int64_t last_call_time);

#endif