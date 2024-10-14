// Include the standard SDK libraries
#include "common_microros_libs.h"

// Include multicore programming
#include "pico/multicore.h"

// Include the microros nodes
#include "sensor_transmission.h"
#include "rudder_control.h"
#include "winch_control.h"

// Change when adding new nodes
#define NUMBER_OF_NODES 1

// Global microros structs
rclc_executor_t executor_core0;
rcl_allocator_t allocator_core0;
rclc_support_t support_core0;

rclc_executor_t executor_core1;
rcl_allocator_t allocator_core1;
rclc_support_t support_core1;




void main_core1() {
    allocator_core1 = rcl_get_default_allocator();

    // Wait for agent successful ping
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rclc_support_init(&support_core1, 0, NULL, &allocator_core1);
    rclc_executor_init(&executor_core1, &support_core1.context, 1, &allocator_core1);

    // ADD YOUR NODES FOR CORE1 HERE!

    // rudder_control(&support_core1, &executor_core1);
    // manual_mode(&support_core1, &executor_core1);
    // semiautomatic_mode(&support_core1, &executor_core1);


    while (true){
        rclc_executor_spin_some(&executor_core1, RCL_MS_TO_NS(100));
    }
}


int main()
{
    stdio_init_all();

    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, 1);

    allocator_core0 = rcl_get_default_allocator();

    // Wait for agent successful ping
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rclc_support_init(&support_core0, 0, NULL, &allocator_core0);
    rclc_executor_init(&executor_core0, &support_core0.context, 1, &allocator_core0);

    // ADD YOUR NODES FOR CORE0 HERE!

    sensor_transmission(&support_core0, &executor_core0);
    //winch_control(&support_core0, &executor_core0);


    // multicore_launch_core1(core1_entry);

    while (true){
        rclc_executor_spin_some(&executor_core0, RCL_MS_TO_NS(100));
    }
    return 0;
}