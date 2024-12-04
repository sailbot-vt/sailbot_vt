#include "rudder_control.h"
#include "pololu_HAL.c"
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

rcl_subscription_t rudder_angle_subscriber;
rcl_publisher_t magnetometer_angle_publisher;

std_msgs__msg__Float32 magnetometer_angle_msg;
std_msgs__msg__Float32 rudder_angle_msg;

static drv8711 rudderDriver;
static amt22 rudderEncoder;
static cmps14 compass;

static float desiredRudderAngleSynch;

void rudder_control_init(rcl_allocator_t *allocator, rclc_support_t *support, rclc_executor_t *executor)
{
    rclc_node_init_default(&rudder_control_node, "rudder_control", "", support);

    rclc_subscription_init_default(
        &rudder_angle_subscriber,
        &rudder_control_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/actions/sail_angle"
    );

    rclc_publisher_init_default(
        &magnetometer_angle_publisher,
        &rudder_control_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/heading"
    );

    rclc_timer_init_default(
        &sensor_transmission_timer,
        support,
        10,
        rudder_control_callback
    );

    rclc_executor_add_timer(executor, &sensor_transmission_timer);

    spi_init(SPI_PORT, 1000 * 1000);
    gpio_set_function(SCLK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MISO_PIN, GPIO_FUNC_SPI);

    gpio_put(LED_PIN, 1);
    AMT22_init(&rudderEncoder, 22, spi1);

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

    cmps14_init(&compass, i2c0, 0x60);

    rclc_executor_add_subscription(executor, &rudder_angle_subscriber, &rudder_angle_msg, &sensor_transmission_timer_callback, ON_NEW_DATA);

    rudder_angle_msg.data = 0.0;
    magnetometer_angle_msg.data = 0.0;
}


void sensor_transmission_timer_callback(const void *msg_in)
{
    
    // float current_rudder_angle = get_motor_angle(&rudderEncoder) - MOTOR_ANGLE_OFFSET;
    // Transform motor angle from (0, 360) to (-180, 180)
    // if (current_rudder_angle >= 180) 
    //     current_rudder_angle -= 360;

    // Serial.println(motor_angle);
    // if (motor_angle <= -55)
    //   motor_angle = -55;

    // a certain turn in the motor does not coorespond to the same turn in the actual rudder, so we use a polynomial to estimate
    // float rudder_angle = -0.00002094 * pow(motor_angle, 3) + 0.001259 * pow(motor_angle, 2) + 0.4159 * motor_angle - 8.373;  // this line is based on the rudder polynomial Adam made (https://media.discordapp.net/attachments/1211815846008193024/1211815857261518848/image.png?ex=65ef9276&is=65dd1d76&hm=1249d54698a71872c4ddfb9c47bb3d35840c5275328245a1d97be5e6cfbd0f59&=&format=webp&quality=lossless&width=1440&height=598)
    // float rudder_angle = -0.00002094 * pow(motor_angle, 3) + 0.001259 * pow(motor_angle, 2) + 0.4159 * motor_angle - 8.373;  // this line is based on the rudder polynomial Adam made (https://media.discordapp.net/attachments/1211815846008193024/1211815857261518848/image.png?ex=65ef9276&is=65dd1d76&hm=1249d54698a71872c4ddfb9c47bb3d35840c5275328245a1d97be5e6cfbd0f59&=&format=webp&quality=lossless&width=1440&height=598)

    // Closed Feedback Loop
    // magnetometer_angle_msg.data = current_rudder_angle;
    const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msg_in;
    // float desired_rudder_angle = msg->data;
    
    desiredRudderAngleSynch = 0.001345 * pow(msg->data, 3) - 0.003741 * pow(msg->data, 2) + 2.142 * msg->data + 19.71;
}
 
void rudder_control_callback() {
    magnetometer_angle_msg.data = 44444.0;
    rcl_publish(&magnetometer_angle_publisher, &magnetometer_angle_msg, NULL);
    float current_rudder_angle = get_motor_angle(&rudderEncoder) - MOTOR_ANGLE_OFFSET;
    magnetometer_angle_msg.data = 55555.0;
    rcl_publish(&magnetometer_angle_publisher, &magnetometer_angle_msg, NULL);
    if (current_rudder_angle >= 180) 
    current_rudder_angle -= 360;

    float rudder_error = current_rudder_angle - desiredRudderAngleSynch;

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
    magnetometer_angle_msg.data = current_rudder_angle;
    rcl_publish(&magnetometer_angle_publisher, &magnetometer_angle_msg, NULL);
}