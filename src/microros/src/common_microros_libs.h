// Microros SDK Librareis
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <rmw_microros/rmw_microros.h>

// Pico SDK libraries
#include <pico/stdlib.h>
#include <pico_uart_transports.h>

// Important constants
#define LED_PIN 25

// Pin declarations
#define SDA_PIN 4
#define SCL_PIN 5

// Port declarations
#define I2C_PORT i2c
