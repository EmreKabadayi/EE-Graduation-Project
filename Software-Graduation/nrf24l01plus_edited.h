#ifndef NRF24L01PLUS_H
#define NRF24L01PLUS_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pico/stdlib.h>
#include "hardware/spi.h"
#include "hardware/timer.h"
#include "hardware/watchdog.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/uart.h"

#define SPI_PORT spi0


typedef struct {
    //Declarations of variables that hold information about the nrf object. 
    //Further information about every variable is given in the related source file.
    uint8_t config_data;
    uint8_t en_aa_data;
    uint8_t en_pipe_data;
    uint8_t aw_data;
    uint8_t retrans_data;
    uint8_t rf_setup_data;
    uint8_t feature_data;
    uint8_t dynamic_payload_data;
    uint8_t p0_address;
    uint8_t p0_payload_width;
    uint8_t transmit_address;

    //A simple bool to check wheter the object acts as a PTX or a PRX.
    bool is_ptx;

    //Spi pin variable declarations.
    uint8_t miso_pin;
    uint8_t cs_pin;
    uint8_t sck_pin;
    uint8_t mosi_pin;
    uint8_t ce_pin;

    //Buffers for answers from slaves.
    uint8_t led_answer;
    uint8_t window_answer;
    uint8_t curtain_answer;
    uint8_t door_answer;

    //Buffers for messages for slaves.
    uint8_t led_message;
    uint8_t window_message;
    uint8_t curtain_message;
    uint8_t door_message;

    //Flags for nrf devices
    bool led_flag;
    bool window_flag;
    bool curtain_flag;
    bool door_flag;

    //Status data from nrf module.
    volatile uint8_t status_nrf;

}nrf_spi_t;

//Addresses of control registers
#define RX_CONFIG_ADDR     0X00
#define EN_AA_ADDR         0x01
#define EN_PIPE_ADDR       0x02 
#define ADDRESS_WIDTH_ADDR 0x03 
#define RETRANS_ADDR       0x04
#define RF_SETUP_ADDR      0x06
#define STATUS_REG_ADDR    0X07
#define FEATURE_ADDR       0x1D 
#define DYNAMIC_PAYLD_ADDR 0x1C 
#define TX_REG_ADDR        0x10
#define P0_ADDR_REG_ADDR   0x0A
#define FIFO_STATUS_ADDR   0x17
#define P0_PW_ADDR         0x11

//PIN DEFINITIONS
#define PTX_MISO 0
#define PTX_CS   1
#define PTX_CLK  2
#define PTX_MOSI 3
#define PTX_CE   4

#define PRX_MISO 16
#define PRX_CS   17
#define PRX_CLK  18
#define PRX_MOSI 19
#define PRX_CE   20

//Various macros for spi commands defined in nrf24l01+ datasheet.
#define WRITE_TX_FIFO 0b10100000
#define READ_RX_FIFO  0b01100001
#define NOP           0b11111111
#define FLUSH_TX      0b11100001
#define FLUSH_RX      0b11100010
#define RX_ACK_PYLOAD 0b10101000 

//Slave board address macros.
#define LED    (uint8_t)0X38
#define WINDOW (uint8_t)0X39
#define STEP   (uint8_t)0X3A
#define DOOR   (uint8_t)0X3B

//CONFIG register values for PTX and PRX modes.
#define CONF_PTX 0b01111010
#define CONF_PRX 0b01111011


//Default configuration bytes that are the same for both PRX and PTX
#define DEFAULT_EN_AA         0b00000001
#define DEFAULT_EN_PIPE       0b00000001
#define DEFAULT_AW_DATA       0b00000011
#define DEFAULT_RETRANS_DATA  0b01010001
#define DEFAULT_DYN_PYLD_DATA 0b00000001
#define DEFAULT_FEATURE_DATA  0b00000111
#define DEFAULT_P0_PW         0b00001111
#define DEFAULT_SETUP_DATA    0b00100000


void 
set_nrf_struct(nrf_spi_t *nrf, bool ptx, uint8_t DeviceAddress);

void 
configure_nrf(nrf_spi_t nrf);

uint8_t 
read_rx_payload_byte();

void 
write_tx_payload_byte(uint8_t buf);

uint8_t 
read_nrf_reg(const uint8_t reg_addr_tess);

void 
write_nrf_reg(const uint8_t reg_addr_tess, const uint8_t reg_data_t);

void 
ce_active();

void 
flush_tx_nrf();

void 
ce_deactive();

void 
cs_select();

void 
cs_deselect();

uint8_t
read_status();

void
ptx_set_transmit_address(nrf_spi_t nrf);

void
flush_tx_nrf();

void
flush_rx_nrf();

uint8_t 
read_fifo_status();

#endif