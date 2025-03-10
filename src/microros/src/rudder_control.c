#include "rudder_control.h"
#include "polulu_HAL.c"
#include "magnetometer_HAL.c"
#include "encoder_read.c"
#include <math.h>

#define ACCEPTABLE_RUDDER_ERROR 1
#define MOTOR_ANGLE_OFFSET 36
#define RUDDER_GAIN 300
#define MAX_RUDDER_CURRENT 2000
#define MIN_TIME_BETWEEN_MOTOR_STEPS 2000 // microseconds

#define CLOCKWISE 1
#define COUNTER_CLOCKWISE 0

#define MAX_RUDDER_ANGLE 35
#define MIN_RUDDER_ANGLE -35

const float MID_RUDDER_ANGLE = (MAX_RUDDER_ANGLE + MIN_RUDDER_ANGLE) / 2;
const int MAX_RUDDER_ERROR = (MAX_RUDDER_ANGLE - MIN_RUDDER_ANGLE);

#define STEP_PERIOD_US 2000

rcl_node_t  rudder_control_node;
rmw_qos_profile_t best_effort_qos_profile;

rcl_timer_t rudder_control_loop_timer;
rcl_subscription_t zero_rudder_encoder_subscriber;
rcl_subscription_t should_relay_be_open_subscriber;
rcl_subscription_t desired_rudder_angle_subscriber;
rcl_publisher_t    current_rudder_angle_publisher;
rcl_publisher_t    current_rudder_motor_angle_publisher;
rcl_publisher_t    magnetometer_angle_publisher;


std_msgs__msg__Bool           should_relay_be_open_msg;  
std_msgs__msg__Float32        magnetometer_angle_msg;
std_msgs__msg__Float32        current_rudder_motor_angle_msg;
std_msgs__msg__Float32        current_rudder_angle_msg;
std_msgs__msg__Float32        desired_rudder_angle_msg;
std_msgs__msg__Bool           empty_request_msg;

static drv8711 rudderDriver;
static amt22   rudderEncoder;
static cmps14  compass;


static float desired_rudder_angle = 0;
static float desired_rudder_motor_angle  = 0;


void rudder_control_init(rcl_allocator_t *allocator, rclc_support_t *support, rclc_executor_t *executor) {
    RCCHECK(rclc_node_init_default(&rudder_control_node, "rudder_control", "", support));


    RCCHECK(rclc_subscription_init_default(
        &zero_rudder_encoder_subscriber,
        &rudder_control_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/zero_rudder_encoder"
    ));
    
    RCCHECK(rclc_subscription_init_default(
        &should_relay_be_open_subscriber,
        &rudder_control_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/should_relay_be_open"
    ));
    
    RCCHECK(rclc_subscription_init_best_effort(
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
        &current_rudder_motor_angle_publisher,
        &rudder_control_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/current_rudder_motor_angle"
    ));

    RCCHECK(rclc_publisher_init_default(
        &magnetometer_angle_publisher,
        &rudder_control_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/heading"
    ));


    RCCHECK(rclc_timer_init_default(
        &rudder_control_loop_timer,
        support,
        RCL_MS_TO_NS(5),
        rudder_control_loop
    ));




    RCCHECK(rclc_executor_add_subscription(
        executor, 
        &desired_rudder_angle_subscriber, 
        &desired_rudder_angle_msg, 
        &desired_rudder_angle_received_callback, 
        ON_NEW_DATA
    ));


    RCCHECK(rclc_executor_add_subscription(
        executor, 
        &should_relay_be_open_subscriber, 
        &should_relay_be_open_msg, 
        &should_relay_be_open_callback, 
        ON_NEW_DATA
    ));


    RCCHECK(rclc_executor_add_subscription(
        executor, 
        &zero_rudder_encoder_subscriber, 
        &should_relay_be_open_msg,
        &zero_rudder_encoder_callback, 
        ON_NEW_DATA
    ));


    RCCHECK(rclc_executor_add_timer(executor, &rudder_control_loop_timer));




    // Initialize Encoder and Motor Controller
    spi_init(SPI_PORT, 500 * 1000);
    gpio_set_function(SCLK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MISO_PIN, GPIO_FUNC_SPI);

    AMT22_init(&rudderEncoder, CS_RE_PIN, SPI_PORT);

    drv8711_init(&rudderDriver, CS_RM_PIN, SLP_RM_PIN, SPI_PORT);
    drv8711_clearStatus(&rudderDriver);
    drv8711_setDecayMode(&rudderDriver, AutoMixed);
    drv8711_setCurrent(&rudderDriver, 2000);
    drv8711_setStepMode(&rudderDriver, MicroStep4);
    drv8711_enableDriver(&rudderDriver);
    drv8711_setAwake(&rudderDriver);


    // Initialize Compass 
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    cmps14_init(&compass, I2C_PORT, 0x60);


    // Initialize Relay
    gpio_init(RELAY_PIN);
    gpio_set_dir(RELAY_PIN, GPIO_OUT);
    gpio_put(RELAY_PIN, 0);

    gpio_init(RELAY_SEL0_PIN);
    gpio_set_dir(RELAY_SEL0_PIN, 1);
    gpio_pull_down(RELAY_SEL0_PIN);
    gpio_init(RELAY_PWM_PIN);
    gpio_set_dir(RELAY_PWM_PIN, 1);
    gpio_pull_down(RELAY_PWM_PIN);
    gpio_init(IN_A_PIN);
    gpio_set_dir(IN_A_PIN, 1);
    gpio_init(IN_B_PIN);
    gpio_set_dir(IN_B_PIN, 1);

    gpio_put(RELAY_SEL0_PIN, 1);
    gpio_put(RELAY_PWM_PIN, 1);

    desired_rudder_angle_msg.data = 0.0;
    current_rudder_angle_msg.data = 0.0;
    magnetometer_angle_msg.data = 0.0;
}


inline float get_rudder_angle_from_motor_angle(float motor_angle) {
    return -0.00002094 * pow(motor_angle, 3) + 0.001259 * pow(motor_angle, 2) + 0.4159 * motor_angle - 8.373;
}

inline float get_motor_angle_from_rudder_angle(float rudder_angle) {
    return 0.001345 * pow(rudder_angle, 3) + 0.003741 * pow(rudder_angle, 2) + 2.142 * rudder_angle + 19.71;
}


void desired_rudder_angle_received_callback(const void *msg_in) {
    const std_msgs__msg__Float32 *desired_rudder_angle_msg = (const std_msgs__msg__Float32 *)msg_in;
    desired_rudder_angle = desired_rudder_angle_msg->data;

    if (desired_rudder_angle > MAX_RUDDER_ANGLE) 
        desired_rudder_angle = MAX_RUDDER_ANGLE;

    if (desired_rudder_angle < MIN_RUDDER_ANGLE)
        desired_rudder_angle = MIN_RUDDER_ANGLE;

    desired_rudder_motor_angle = get_motor_angle_from_rudder_angle(desired_rudder_angle);
}


void should_relay_be_open_callback(const void *msg_in) {
    const std_msgs__msg__Bool *should_relay_be_open_msg = (const std_msgs__msg__Bool *)msg_in;

    gpio_put(IN_A_PIN, (int) should_relay_be_open_msg->data);
}

void zero_rudder_encoder_callback(const void *msg_in) {
    zero_encoder_value(&rudderEncoder);
}



void rudder_control_loop() {
    float current_rudder_motor_angle = get_motor_angle(&rudderEncoder) + MOTOR_ANGLE_OFFSET;     // motor_angle % 360

    if (current_rudder_motor_angle >= 180) {
        current_rudder_motor_angle -= 360;
    }

    float current_rudder_angle = get_rudder_angle_from_motor_angle(current_rudder_motor_angle);

    float rudder_error = current_rudder_motor_angle - desired_rudder_motor_angle;

    if (abs(rudder_error) > ACCEPTABLE_RUDDER_ERROR) {
        if (((int)rudder_error % 360) > 0 && ((int)rudder_error % 360) < 180) 
            drv8711_setDirection(&rudderDriver, COUNTER_CLOCKWISE);
    
        else 
            drv8711_setDirection(&rudderDriver, CLOCKWISE);

        int number_of_steps = (int)(abs(rudder_error) * RUDDER_GAIN / MAX_RUDDER_ERROR);   

        if (number_of_steps > 50) {
            number_of_steps = 50;
        }

        for (int i = 0; i < number_of_steps; i++) {
            drv8711_step(&rudderDriver);
            sleep_us(MIN_TIME_BETWEEN_MOTOR_STEPS);
        }
    }

    magnetometer_angle_msg.data = cmps14_getBearing(&compass) / 10.0;
    current_rudder_angle_msg.data = current_rudder_angle;
    current_rudder_motor_angle_msg.data = current_rudder_motor_angle;

    rcl_publish(&current_rudder_motor_angle_publisher, &current_rudder_motor_angle_msg, NULL);
    rcl_publish(&current_rudder_angle_publisher, &current_rudder_angle_msg, NULL);
    rcl_publish(&magnetometer_angle_publisher, &magnetometer_angle_msg, NULL);

}