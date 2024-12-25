#ifndef UROS_LOGGER_H
#define UROS_LOGGER_H

#include "common_microros_libs.h"
#include "rmw/qos_profiles.h"

extern rcl_publisher_t uROS_logger_publisher;
extern rcl_interfaces__msg__Log uROS_logger_msg;
extern rcl_node_t uROS_logger_node;

void uros_logger_init(rclc_support_t *support);
void uros_log(const char *msg, uint8_t severity, const char *file, const char *function);

#endif