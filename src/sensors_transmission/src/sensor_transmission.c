#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h" // This will always have a squiggle get used to it
#include "pico_uart_transports.h"

// Can be removed with random stuff
#include <stdlib.h>
#include <time.h>

const uint LED_PIN = 25;

rcl_publisher_t winch_angle_publisher;
rcl_publisher_t rudder_angle_publisher;
rcl_publisher_t magnetometer_angle_publisher;
std_msgs__msg__Int32 winch_angle;
std_msgs__msg__Int32 rudder_angle;
std_msgs__msg__Int32 magnetometer_angle;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    rcl_ret_t winchRet = rcl_publish(&winch_angle_publisher, &winch_angle, NULL);
    rcl_ret_t rudderRet = rcl_publish(&rudder_angle_publisher, &rudder_angle, NULL);
    rcl_ret_t magnetometerRet = rcl_publish(&magnetometer_angle_publisher, &magnetometer_angle, NULL);
    
    // Replace those lines with functions actually getting actual serial data
    winch_angle.data = rand() % 361;
    rudder_angle.data = rand() % 361;
    magnetometer_angle.data = rand() % 361;
}

int main()
{
    // Randomposting timer
    srand(time(0));


    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT); // This will always have a squiggle get used to it

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);
    if (ret != RCL_RET_OK) {
        // Unreachable agent, exiting program
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "sensor_transmission", "", &support);

    rclc_publisher_init_default(
        &winch_angle_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/actions/winch_angle"
    );
    rclc_publisher_init_default(
        &rudder_angle_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/actions/rudder_angle"
    );
    rclc_publisher_init_default(
        &magnetometer_angle_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/actions/heading"
    );

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback
    );

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    gpio_put(LED_PIN, 1);

    winch_angle.data = 0;
    rudder_angle.data = 0;
    magnetometer_angle.data = 0;

    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}
