#include "winch_control.h"
#include "encoder_read.c"

rcl_publisher_t winch_encoder_angle_publisher;
std_msgs__msg__Float32 winch_encoder_angle;

// rcl_publisher_t uROS_logger_publisher;
// rcl_interfaces__msg__Log uROS_logger_msg;

rcl_timer_t winch_control_timer;
rcl_node_t winch_control_node;

static amt22 rudderEncoder;

// void uros_log(const char *msg, uint8_t severity, const char *file, const char *function) {

//     uROS_logger_msg.name.data = "Microros";
//     uROS_logger_msg.name.size = strlen(uROS_logger_msg.name.data);
//     uROS_logger_msg.name.capacity = uROS_logger_msg.name.size + 1;

//     uROS_logger_msg.level = severity;

//     uROS_logger_msg.msg.data = (char *)msg;
//     uROS_logger_msg.msg.size = strlen(uROS_logger_msg.msg.data);
//     uROS_logger_msg.msg.capacity = uROS_logger_msg.msg.size + 1;

//     uROS_logger_msg.file.data = (char *)file;
//     uROS_logger_msg.file.size = strlen(uROS_logger_msg.file.data);;
//     uROS_logger_msg.file.capacity = uROS_logger_msg.file.size + 1;
    
//     uROS_logger_msg.function.data = (char *)function;
//     uROS_logger_msg.function.size = strlen(uROS_logger_msg.function.data);
//     uROS_logger_msg.function.capacity = uROS_logger_msg.function.size;
    

//     rcl_publish(&uROS_logger_publisher, &uROS_logger_msg, NULL);
// }


void winch_control(rcl_allocator_t *allocator, rclc_support_t *support, rclc_executor_t *executor)
{
    rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
    qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;

    rclc_node_init_default(&winch_control_node, "winch_control", "", support);

    rclc_publisher_init_default(
        &winch_encoder_angle_publisher,
        &winch_control_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/winch_angle"
    );

    
    // rclc_publisher_init(
    //     &uROS_logger_publisher,
    //     &winch_control_node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log),
    //     "/rosout",
    //     &qos_profile
    // );
    // rcl_interfaces__msg__Log__init(&uROS_logger_msg);

    rclc_timer_init_default(
        &winch_control_timer,
        support,
        RCL_MS_TO_NS(5),
        winch_control_timer_callback
    );
    
    spi_init(SPI_PORT, 500 * 1000);
    gpio_set_function(SCLK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MISO_PIN, GPIO_FUNC_SPI);

    gpio_put(LED_PIN, 1);

    AMT22_init(&rudderEncoder, 22, SPI_PORT);    
    
    rclc_executor_add_timer(executor, &winch_control_timer);

    winch_encoder_angle.data = 0.0;
}

void winch_control_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    winch_encoder_angle.data = get_motor_angle(&rudderEncoder);
    
    rcl_publish(&winch_encoder_angle_publisher, &winch_encoder_angle, NULL);

}
