#include "sensor_transmission.h"
#include "magnetometer_read.c"

rcl_publisher_t magnetometer_angle_publisher;
std_msgs__msg__Float32 magnetometer_angle;

void sensor_transmission(rclc_support_t *support, rclc_executor_t *executor)
{
    rcl_timer_t timer;
    rcl_node_t node;
    rclc_node_init_default(&node, "sensor_transmission", "", support);

    rclc_publisher_init_default(
        &magnetometer_angle_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/heading"
    );

    rclc_timer_init_default(
        &timer,
        support,
        RCL_MS_TO_NS(100),
        timer_callback
    );

    rclc_executor_add_timer(executor, &timer);


    magnetometer_angle.data = 0.0;
    gpio_put(LED_PIN, 1);
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    i2c_init_custom(0x60);

    int16_t bearing = getBearing();

    if (bearing == -1 || bearing == 0) {
        return;
    }
    
    float bearing_f = bearing / 10.0f;
    magnetometer_angle.data = bearing_f;
}