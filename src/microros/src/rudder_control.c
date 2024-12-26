#include "rudder_control.h"
#include "polulu_HAL.c"
#include "magnetometer_HAL.c"
#include "encoder_read.c"
#include <math.h>

#define ACCEPTABLE_RUDDER_ERROR 1
#define MOTOR_ANGLE_OFFSET 36
#define RUDDER_GAIN 300
#define MAX_RUDDER_SPEED 50
#define MAX_RUDDER_CURRENT 2000

#define CLOCKWISE 1
#define COUNTER_CLOCKWISE 0

#define MAX_RUDDER_ANGLE 20
#define MIN_RUDDER_ANGLE -20

const float MID_RUDDER_ANGLE = (MAX_RUDDER_ANGLE + MIN_RUDDER_ANGLE) / 2;
const int MAX_RUDDER_ERROR = (MAX_RUDDER_ANGLE - MIN_RUDDER_ANGLE);

#define STEP_PERIOD_US 2000

rcl_node_t rudder_control_node;
rcl_timer_t sensor_transmission_timer;

rcl_subscription_t desired_rudder_angle_subscriber;
rcl_publisher_t current_rudder_angle_publisher;
rcl_publisher_t magnetometer_angle_publisher;

std_msgs__msg__Float32 magnetometer_angle_msg;
std_msgs__msg__Float32 current_rudder_angle_msg;
std_msgs__msg__Float32 desired_rudder_angle_msg;

static drv8711 rudderDriver;
static amt22 rudderEncoder;
static cmps14 compass;

static float desired_rudder_angle_synch = 0;

#define RCCHECK(fn) {                   \
    rcl_ret_t temp_rc = fn;             \
    if ((temp_rc != RCL_RET_OK))     \
    {                                   \
        gpio_put(LED_PIN, 1);           \
    }                                   \
}


void rudder_control_init(rcl_allocator_t *allocator, rclc_support_t *support, rclc_executor_t *executor) {
    RCCHECK(rclc_node_init_default(&rudder_control_node, "rudder_control", "", support));

    RCCHECK(rclc_subscription_init_default(
        &desired_rudder_angle_subscriber,
        &rudder_control_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/actions/rudder_angle"
    ));

    RCCHECK(rclc_publisher_init_default(
        &current_rudder_angle_publisher,
        &rudder_control_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/current_rudder_angle"
    ));

    RCCHECK(rclc_publisher_init_default(
        &magnetometer_angle_publisher,
        &rudder_control_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/heading"
    ));


    RCCHECK(rclc_timer_init_default(
        &sensor_transmission_timer,
        support,
        RCL_MS_TO_NS(5),
        rudder_control_callback
    ));

    RCCHECK(rclc_executor_add_timer(executor, &sensor_transmission_timer));

    spi_init(SPI_PORT, 500 * 1000);
    gpio_set_function(SCLK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MISO_PIN, GPIO_FUNC_SPI);

    AMT22_init(&rudderEncoder, 22, SPI_PORT);

    drv8711_init(&rudderDriver, CS_RM_PIN, SLP_RM_PIN, SPI_PORT);
    drv8711_clearStatus(&rudderDriver);
    drv8711_setDecayMode(&rudderDriver, AutoMixed);
    drv8711_setCurrent(&rudderDriver, 2000);
    drv8711_setStepMode(&rudderDriver, MicroStep4);
    drv8711_enableDriver(&rudderDriver);
    drv8711_setAwake(&rudderDriver);

    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    cmps14_init(&compass, I2C_PORT, 0x60);

    desired_rudder_angle_msg.data = 0.0;
    current_rudder_angle_msg.data = 0.0;
    magnetometer_angle_msg.data = 0.0;

    RCCHECK(rclc_executor_add_subscription(executor, &desired_rudder_angle_subscriber, &current_rudder_angle_msg, &desired_rudder_angle_received_callback, ON_NEW_DATA));

}


void desired_rudder_angle_received_callback(const void *msg_in) {
    const std_msgs__msg__Float32 *desired_rudder_angle_msg = (const std_msgs__msg__Float32 *)msg_in;
    desired_rudder_angle_synch = desired_rudder_angle_msg->data;
    
    // desired_rudder_angle_synch = 0.001345 * pow(x, 3) + 0.003741 * pow(x, 2) + 2.142 * x + 19.71;
}
 

void rudder_control_callback() {
    float current_rudder_motor_angle = get_motor_angle(&rudderEncoder) - MOTOR_ANGLE_OFFSET;
    float current_rudder_angle = -2.094e-5 * pow(current_rudder_motor_angle, 3) + 0.001259 * pow(current_rudder_motor_angle, 2) + 0.4159 * pow(current_rudder_motor_angle, 1) - 8.373;

    if (current_rudder_angle >= 180) {
        current_rudder_angle -= 360;
    }

    float rudder_error = current_rudder_angle - desired_rudder_angle_synch;

    if (abs(rudder_error) > ACCEPTABLE_RUDDER_ERROR) {
        if (((int)rudder_error % 360) > 0 && ((int)rudder_error % 360) < 180) 
            drv8711_setDirection(&rudderDriver, COUNTER_CLOCKWISE);
    
        else 
            drv8711_setDirection(&rudderDriver, CLOCKWISE);

        int number_of_steps = (int)(abs(rudder_error) * MAX_RUDDER_SPEED / MAX_RUDDER_ERROR);   
        for (int i = 0; i < number_of_steps; i++) {
            drv8711_step(&rudderDriver);
            sleep_us(2000);
        }
    }

    magnetometer_angle_msg.data = cmps14_getBearing(&compass);
    current_rudder_angle_msg.data = current_rudder_angle;

    rcl_publish(&current_rudder_angle, &magnetometer_angle_msg, NULL);
    rcl_publish(&magnetometer_angle_publisher, &magnetometer_angle_msg, NULL);

}