#include "common_microros_libs.h"
#include "sensor_transmission.h"
#include "magnetometer_read.c"


rcl_publisher_t magnetometer_angle_publisher;

std_msgs__msg__Float32 magnetometer_angle;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    rcl_ret_t magnetometerRet = rcl_publish(&magnetometer_angle_publisher, &magnetometer_angle, NULL);
    
    // Replace those lines with functions actually getting actual serial data
    i2c_init_custom(0x60, 4, 5);

    int16_t bearing = getBearing();

    if (bearing == -1 || bearing == 0) {
        return;
    }
    
    float bearing_f = bearing / 10.0f;
    magnetometer_angle.data = bearing_f;
}

rcl_ret_t sensor_transmission()
{
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, 1); // This will always have a squiggle get used to it

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
        &magnetometer_angle_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/heading"
    );

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(100),
        timer_callback
    );

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    gpio_put(LED_PIN, 1);

    magnetometer_angle.data = 0.0;

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
}
