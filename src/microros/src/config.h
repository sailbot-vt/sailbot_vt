// 0 for the motorboat mode
// 1 for the sailboat mode
#define BOATMODE 0
#define DEBUG 0


// Important constants
#define LED_PIN 25
#define TIMEOUT_MS 1000
#define AGENT_ATTEMPTS 120

// Pin declarations
// UART
#define TX_PIN 0
#define RX_PIN 1

// Debug
#define SWCLK_PIN 2
#define SWDIO_PIN 3
#define SWDTX_PIN 4
#define SWDRX_PIN 5

// I2C
#define SDA_PIN 20
#define SCL_PIN 21

// SPI
#define SCLK_PIN 10
#define MISO_PIN 12
#define MOSI_PIN 11

// RC Channels
#define CH1_PIN 28
#define CH2_PIN 27
#define CH3_PIN 26

// Chip Selections
#define CS_RM_PIN 13
#define CS_WM_PIN 6
#define CS_RE_PIN 7
#define CS_WE_PIN 9

// Miscellaneous
#define SLP_RM_PIN 17
#define SLP_WM_PIN 14
#define RELAY_PIN 22

// Relay
#define RELAY_SEL0_PIN 19
#define RELAY_PWM_PIN 18
#define IN_A_PIN 8
#define IN_B_PIN 16

// Port declarations
#define I2C_PORT i2c0
#define SPI_PORT spi1