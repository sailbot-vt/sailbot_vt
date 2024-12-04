#include "sensor_transmission.h"
#include "magnetometer_HAL.c"

rcl_publisher_t magnetometer_angle_publisher;
std_msgs__msg__Float32 magnetometer_angle_msg;

rcl_timer_t sensor_transmission_timer;
rcl_node_t sensor_transmission_node;

void sensor_transmission_init(rcl_allocator_t *allocator, rclc_support_t *support, rclc_executor_t *executor)
{
    // rmw_uros_ping_agent(TIMEOUT_MS, AGENT_ATTEMPTS);
    rclc_node_init_default(&sensor_transmission_node, "sensor_transmission", "", support);
    rclc_publisher_init_default(
        &magnetometer_angle_publisher,
        &sensor_transmission_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/heading"
    );

    rclc_timer_init_default(
        &sensor_transmission_timer,
        support,
        RCL_MS_TO_NS(100),
        sensor_transmission_timer_callback
    );

    rclc_executor_add_timer(executor, &sensor_transmission_timer);

    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN); // This is what we wanted on the PCB
    gpio_pull_up(SCL_PIN);


    // uros_log("Initialized i2c", 0, "sensor transmission", "initialization");

    magnetometer_angle_msg.data = 0.0;
}

void sensor_transmission_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    int16_t bearing = getBearing();

    if (bearing == -1 || bearing == 0) {
        return;
    }
    
    float bearing_f = bearing / 10.0f;
    magnetometer_angle_msg.data = bearing_f;
    rcl_publish(&magnetometer_angle_publisher, &magnetometer_angle_msg, NULL);

}