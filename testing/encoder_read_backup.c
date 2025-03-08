#include "common_microros_libs.h"
#include "hardware/spi.h"
#include <math.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include <stdlib.h>


#include <stdio.h>
#include <string.h>


#define READ_BIT 0x00



#define READ_RATE 10000
#define spi1 ((spi_inst_t *)spi1_hw)

uint8_t NO_OP = 0x00;
const uint8_t RESET_ENCODER = 0x60;
const uint8_t SET_ZERO_POINT = 0x70;
const uint8_t READ_TURNS = 0xA0;


//spi0 pins
//SCK 18 White - Purple
// TX 19 Blue - Grey
// RX 16 Green - Brown
// CSN 17 Yellow - black

// Black - White
// Red - Blue

//SPI1 pins
//SCK 10 White - Purple
// TX (MOSI) 11 Blue - Grey
// RX (MISO) 12 Green - Brown
// CSN 13 Yellow - black


#define PICO_SPI_SCK_PIN 10

// #define PICO_SPI_CSN_PIN 26

#define PICO_SPI_TX_PIN 11

#define PICO_SPI_RX_PIN 12

int PICO_SPI_CSN_PIN;

float turn_count = 0;

//Blue is MOSI 
//Green is MISO

float cur_angle = 0;


void AMTT22_Encoder(int pin){
    PICO_SPI_CSN_PIN = pin;
    stdio_init_all();


    spi_init(spi1, 500 * 1000);
    gpio_set_function(PICO_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_SPI_TX_PIN, GPIO_FUNC_SPI);
    // Make the SPI pins available to picotool
    // bi_decl(bi_3pins_with_func(PICO_SPI_RX_PIN, PICO_SPI_TX_PIN, PICO_SPI_SCK_PIN, GPIO_FUNC_SPI));

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(PICO_SPI_CSN_PIN);
    gpio_set_dir(PICO_SPI_CSN_PIN, GPIO_OUT);
    gpio_put(PICO_SPI_CSN_PIN, 1);
    gpio_pull_up(PICO_SPI_CSN_PIN);
    // Make the CS pin available to picotool
    // bi_decl(bi_1pin_with_name(PICO_SPI_CSN_PIN, "SPI CS"));

}


int get_turn_count(){
    return turn_count;
    
}

static inline void cs_select() {
    asm volatile("nop \n nop \n nop");
    asm volatile("nop \n nop \n nop");
    gpio_put(PICO_SPI_CSN_PIN, 0);  // Active low
    asm volatile("nop \n nop \n nop");
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect() {
    asm volatile("nop \n nop \n nop");
    asm volatile("nop \n nop \n nop");
    gpio_put(PICO_SPI_CSN_PIN, 1);
    asm volatile("nop \n nop \n nop");
    asm volatile("nop \n nop \n nop");
}


void zero_encoder_value(){
    sleep_us(40);
    gpio_put(PICO_SPI_CSN_PIN, 0);
    sleep_us(3);
    spi_write_blocking(spi1, &SET_ZERO_POINT,8);
    sleep_us(3);
    gpio_put(PICO_SPI_CSN_PIN, 1);    
}

uint8_t* read_position(uint8_t * bytes_read){
    sleep_us(40);    
    cs_select();
    sleep_us(3);
    uint8_t send[2] = {0x00, 0x00};    
    spi_write_read_blocking(spi1,send,bytes_read,2);
    sleep_us(3);

    cs_deselect();
    
    return bytes_read;

}

// uint8_t* read_position_multiturn(){
//     sleep_us(40);
//     gpio_put(PICO_SPI_CSN_PIN, 0);
//     static uint8_t bytes_read[4] = {0x00, 0x00, 0x00,0x00};
//     sleep_us(3);
//     spi_write_read_blocking(spi1, &NO_OP, &bytes_read[0],8);
//     sleep_us(3);
//     spi_write_read_blocking(spi1, &READ_TURNS, &bytes_read[1],8);
//     sleep_us(3);
//     spi_write_read_blocking(spi1, &NO_OP, &bytes_read[2],8);
//     sleep_us(3);
//     spi_write_read_blocking(spi1, &NO_OP, &bytes_read[3],8);
//     sleep_us(3);

//     gpio_put(PICO_SPI_CSN_PIN, 1);

//     return bytes_read;

// }

bool get_bit(uint8_t byte, int index){
    return (byte & 1 << (index)) != 0;

}

bool verify_packet(uint8_t packet_contents[2]){
    uint8_t first_byte = packet_contents[0];
    uint8_t second_byte = packet_contents[1];

    bool odd_parity = get_bit(first_byte, 7);
    bool even_parity = get_bit(first_byte, 6);

    bool odd_bits[7] = {
        get_bit(first_byte, 1), 
        get_bit(first_byte, 3),
        get_bit(first_byte, 5),
        get_bit(second_byte, 1),
        get_bit(second_byte, 3),
        get_bit(second_byte, 5),
        get_bit(second_byte, 7)
    };

    bool even_bits[7] = {
        get_bit(first_byte, 0), 
        get_bit(first_byte, 2),
        get_bit(first_byte, 4),
        get_bit(second_byte, 0),
        get_bit(second_byte, 2),
        get_bit(second_byte, 4),
        get_bit(second_byte, 6)
    };

     if (odd_parity == (odd_bits[0] ^ odd_bits[1] ^ odd_bits[2] ^ odd_bits[3] ^ odd_bits[4] ^ odd_bits[5] ^ odd_bits[6])) {
        return false;
    }
    if (even_parity == (even_bits[0] ^ even_bits[1] ^ even_bits[2] ^ even_bits[3] ^ even_bits[4] ^ even_bits[5] ^ even_bits[6])) {
        return false;
    }
    return true;
}

float parse_angle(uint8_t packet_contents[2]){

    packet_contents[0] = packet_contents[0] & ~0b11000000;
    uint16_t angle_raw =packet_contents[0];
    angle_raw <<= 8;
    angle_raw |= packet_contents[1];

    angle_raw >>= 2; 
    float angle = ((float)angle_raw * 360) / (float)(pow(2, 12));

    return angle;
}

float get_motor_angle(){
    sleep_us(READ_RATE);

    uint8_t* packet_array = (uint8_t*) malloc(2*sizeof(uint8_t));
    
    read_position(packet_array) ;
    if (verify_packet(packet_array) != 1){return cur_angle;} 
    // if (verify_packet(packet_array) != 1){return -1;} 

    float next_angle = parse_angle(packet_array);
    
    free(packet_array);

    if (cur_angle > 270 && next_angle <90) {turn_count += 1;}
    if (cur_angle < 90 && next_angle > 270) {turn_count -= 1;}

    cur_angle = next_angle;
    return cur_angle;
}
