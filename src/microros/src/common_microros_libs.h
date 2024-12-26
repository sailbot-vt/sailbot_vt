// Microros SDK Librareis
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>
#include <rmw_microros/rmw_microros.h>
#include <rcl_interfaces/msg/log.h>

// Pico SDK libraries
#include <pico/stdlib.h>
#include <pico_uart_transports.h>

// Important constants
#define LED_PIN 25
#define TIMEOUT_MS 1000
#define AGENT_ATTEMPTS 120

// Pin declarations
// I2C
#define SDA_PIN 4
#define SCL_PIN 5

// SPI
#define SCLK_PIN 10
#define MISO_PIN 12
#define MOSI_PIN 11

// Chip Selections
#define CS_RM_PIN 13
#define CS_WM_PIN 15
#define CS_RE_PIN 22
#define CS_WE_PIN 26

// Miscellaneous
#define SLP_RM_PIN 19
#define SLP_WM_PIN 14
#define MOTOCTL 27

// Port declarations
#define I2C_PORT i2c0
#define SPI_PORT spi1