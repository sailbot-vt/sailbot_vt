// Include the standard SDK libraries
#include "common_microros_libs.h"

// Include multicore programming
#include "pico/multicore.h"

// Include the microros nodes
#include "sensor_transmission.h"
#include "rudder_control.h"
#include "winch_control.h"

// void core1_entry() {
//     motor_control()
// }


int main()
{
    stdio_init_all();

    sensor_transmission();

    while (true){
        // multicore_launch_core1(core1_entry);
        
    }

    return 0;
}