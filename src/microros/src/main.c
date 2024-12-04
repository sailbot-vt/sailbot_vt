// Include the standard SDK libraries
#include "common_microros_libs.h"
// Include multicore programming
#include "pico/multicore.h"

// Include the microros nodes
#include "rudder_control.h"

// Change when adding new nodes
#define EXECUTOR_CORE0_HANDLES 10
#define EXECUTOR_CORE1_HANDLES 1

// Global variables
volatile bool core1_ready = false;



void main_core1() {

    // We don't wanna debug multithreading hazards
    rcl_allocator_t allocator_core1 = rcl_get_default_allocator();
    rclc_executor_t executor_core1;
    rclc_support_t support_core1;

    rclc_support_init(&support_core1, 0, NULL, &allocator_core1);
    rclc_executor_init(&executor_core1, &support_core1.context, EXECUTOR_CORE1_HANDLES, &allocator_core1);

    core1_ready = true;

    // ADD YOUR NODES FOR CORE1 HERE!

    // rudder_control(&support_core1, &executor_core1);
    // manual_mode(&support_core1, &executor_core1);
    // semiautomatic_mode(&support_core1, &executor_core1);


    while (true){
        rclc_executor_spin_some(&executor_core1, RCL_MS_TO_NS(100));
    }
    // CLEAN UP CORE1 NODES HERE!
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
    gpio_set_dir(LED_PIN, GPIO_OUT);

    rcl_allocator_t allocator_core0 = rcl_get_default_allocator();
    rclc_executor_t executor_core0;
    rclc_support_t support_core0;

    // Wait for a successful ping
    rmw_uros_ping_agent(TIMEOUT_MS, AGENT_ATTEMPTS);

    rclc_support_init(&support_core0, 0, NULL, &allocator_core0);
    rclc_executor_init(&executor_core0, &support_core0.context, EXECUTOR_CORE0_HANDLES, &allocator_core0);
    // uros_logger_init(&support_core0);

    // ADD YOUR NODES FOR CORE0 HERE!
    // sensor_transmission_init(&allocator_core0, &support_core0, &executor_core0);
    
    rudder_control_init(&allocator_core0, &support_core0, &executor_core0);


    // Multicore does not work yet
    // multicore_launch_core1(main_core1);

    // Wait until connection with core1 is established
    // while (!core1_ready) {
    //     tight_loop_contents();
    // }


    // reset_usb_boot(0, 0);

    // gpio_put(LED_PIN, 1);

    while (true) {
        rclc_executor_spin_some(&executor_core0, 10);//RCL_MS_TO_NS(1));
    }

    // CLEAN UP CORE0 NODES HERE!

    return 0;
}