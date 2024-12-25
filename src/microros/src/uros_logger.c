#include "uros_logger.h"

rcl_publisher_t uROS_logger_publisher;
rcl_interfaces__msg__Log uROS_logger_msg;
rcl_node_t uROS_logger_node;

void uros_logger_init(rclc_support_t *support) {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
    qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;

    rclc_node_init_default(&uROS_logger_node, "uROS_logger", "", support);

    rclc_publisher_init(
        &uROS_logger_publisher,
        &uROS_logger_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log),
        "/rosout",
        &qos_profile
    );
    rcl_interfaces__msg__Log__init(&uROS_logger_msg);
}

void uros_log(const char *msg, uint8_t severity, const char *file, const char *function) {

    uROS_logger_msg.name.data = "Microros";
    uROS_logger_msg.name.size = strlen(uROS_logger_msg.name.data);
    uROS_logger_msg.name.capacity = uROS_logger_msg.name.size + 1;

    uROS_logger_msg.level = severity;

    uROS_logger_msg.msg.data = (char *)msg;
    uROS_logger_msg.msg.size = strlen(uROS_logger_msg.msg.data);
    uROS_logger_msg.msg.capacity = uROS_logger_msg.msg.size + 1;

    uROS_logger_msg.file.data = (char *)file;
    uROS_logger_msg.file.size = strlen(uROS_logger_msg.file.data);;
    uROS_logger_msg.file.capacity = uROS_logger_msg.file.size + 1;
    
    uROS_logger_msg.function.data = (char *)function;
    uROS_logger_msg.function.size = strlen(uROS_logger_msg.function.data);
    uROS_logger_msg.function.capacity = uROS_logger_msg.function.size;
    

    rcl_publish(&uROS_logger_publisher, &uROS_logger_msg, NULL);
}
