#include "nrf24l01plus.h"
static uint8_t g_cs_pin;
static uint8_t g_ce_pin;

/*
@brief
Sets the nrf struct for operation based on given parameters.

@param
param[in] nrf NRF object to be set
param[in] ptx Binary value. 1 if device is wanted to be PTX, 0 is device is wanted to be PRX.
param[in] DeviceAddress Address of the device if it is wanted to be a PRX.

*/
void set_nrf_struct(nrf_spi_t *const nrf, bool ptx, uint8_t DeviceAddress)
{
    if(ptx)
    {
        nrf->config_data   = CONF_PTX;
        nrf->is_ptx   = true;
        nrf->miso_pin = PTX_MISO;
        nrf->cs_pin   = PTX_CS;
        nrf->sck_pin  = PTX_CLK;
        nrf->mosi_pin = PTX_MOSI;
        nrf->ce_pin   = PTX_CE;
    }
    else
    {
        nrf->config_data   = CONF_PRX;
        nrf->p0_address    = DeviceAddress;
        nrf->is_ptx   = false;
        nrf->miso_pin = PRX_MISO;
        nrf->cs_pin   = PRX_CS;
        nrf->sck_pin  = PRX_CLK;
        nrf->mosi_pin = PRX_MOSI;
        nrf->ce_pin   = PRX_CE;
    }
    // Common configuration register data.
    nrf->en_aa_data           = DEFAULT_EN_AA; //Enable AutoAcknowledgement on pipe 0.
    nrf->en_pipe_data         = DEFAULT_EN_PIPE; // Enable Pipe 0.
    nrf->aw_data              = DEFAULT_AW_DATA; //Adress width is 5 bytes.
    nrf->retrans_data         = DEFAULT_RETRANS_DATA; // 1500us delay. 1 retransmit.
    nrf->dynamic_payload_data = DEFAULT_DYN_PYLD_DATA; //Enables Dynamic Payload on pipe 0.
    nrf->feature_data         = DEFAULT_FEATURE_DATA; //Enables Dynamic payload and AA.
    nrf->p0_payload_width     = DEFAULT_P0_PW; //A lot of bytes.
    nrf->rf_setup_data        = DEFAULT_SETUP_DATA; //-18dB. 250kbps.
    g_cs_pin = nrf->cs_pin;
    g_ce_pin = nrf->ce_pin;

}

/*
@brief
Sets and configures nrf24l01 device for transmission.
Check datasheet and header for more information.

@param
param[in] nrf Object for nrf
*/
void 
configure_nrf(nrf_spi_t nrf)
{
    write_nrf_reg(RX_CONFIG_ADDR,nrf.config_data);
    busy_wait_us_32(10000);
    write_nrf_reg(EN_AA_ADDR,nrf.en_aa_data);
    busy_wait_us_32(10000);
    write_nrf_reg(EN_PIPE_ADDR,nrf.en_pipe_data);
    busy_wait_us_32(10000);
    write_nrf_reg(ADDRESS_WIDTH_ADDR,nrf.aw_data);
    busy_wait_us_32(10000);
    write_nrf_reg(RETRANS_ADDR,nrf.retrans_data);
    busy_wait_us_32(10000);
    write_nrf_reg(RF_SETUP_ADDR,nrf.rf_setup_data);
    busy_wait_us_32(10000);
    write_nrf_reg(FEATURE_ADDR,nrf.feature_data);
    busy_wait_us_32(10000);
    write_nrf_reg(DYNAMIC_PAYLD_ADDR,nrf.dynamic_payload_data);
    busy_wait_us_32(10000);
    write_nrf_reg(P0_PW_ADDR,nrf.p0_payload_width);
    busy_wait_us_32(10000);
    if(!(nrf.is_ptx))
    {
    write_nrf_reg(P0_ADDR_REG_ADDR,nrf.p0_address);
    busy_wait_us_32(10000);
    }
}

/*
@brief
Disables the SPI device its attached to.
*/
void 
cs_deselect()
{
    gpio_put(g_cs_pin,1);
}

/*
@brief
Enables the SPI device its attached to.
*/
void 
cs_select()
{
    gpio_put(g_cs_pin,0);
}

/*
@brief
Deactivates CE pin.
*/
void 
ce_deactive()
{
    gpio_put(g_ce_pin,0);
}

/*
@brief
Activates CE pin.
*/
void 
ce_active()
{
    gpio_put(g_ce_pin,1);
}

/*
@brief
Writes to nRF24L01+ device in its own format.
param[in]  reg_address   address of the register that is wanted to be written to.
param[in]  reg_data      data that is wanted to be written on the given address.
*/
void 
write_nrf_reg(const uint8_t reg_address, const uint8_t reg_data)
{
    uint8_t regAddressBuffer = ((0b00100000) + reg_address);
    cs_select();
    spi_write_blocking(spi0,&regAddressBuffer,sizeof(regAddressBuffer));
    spi_write_blocking(spi0,&reg_data,sizeof(reg_data));
    cs_deselect();
}

/*
@brief
Sets the PTX device's transmission address to the "transmit_address" value of the nrf object.

@param
param[in] nrf object for the nrf device

*** NRF OBJECT'S "transmit_address" MEMBER MUST BE CHANGED BEFORE THIS OPERATION ***

*/
void
ptx_set_transmit_address(nrf_spi_t nrf)
{   
    uint8_t tx_addr_buf     = ((0b00100000) + TX_REG_ADDR);
    uint8_t p0_addr_buf     = ((0b00100000) + P0_ADDR_REG_ADDR);
    uint8_t device_addr_buf = nrf.transmit_address;
    cs_select();
    spi_write_blocking(spi0,&tx_addr_buf,sizeof(uint8_t));
    spi_write_blocking(spi0,&device_addr_buf,sizeof(uint8_t));
    spi_write_blocking(spi0,(uint8_t) '0',sizeof(uint8_t));
    spi_write_blocking(spi0,(uint8_t) '0',sizeof(uint8_t));
    spi_write_blocking(spi0,(uint8_t) '0',sizeof(uint8_t));
    spi_write_blocking(spi0,(uint8_t) '0',sizeof(uint8_t));
    cs_deselect();
    busy_wait_us_32(200);
    cs_select();
    spi_write_blocking(spi0,&p0_addr_buf,sizeof(uint8_t));
    spi_write_blocking(spi0,&device_addr_buf,sizeof(uint8_t));
    spi_write_blocking(spi0,(uint8_t) '0',sizeof(uint8_t));
    spi_write_blocking(spi0,(uint8_t) '0',sizeof(uint8_t));
    spi_write_blocking(spi0,(uint8_t) '0',sizeof(uint8_t));
    spi_write_blocking(spi0,(uint8_t) '0',sizeof(uint8_t));
    cs_deselect();
}

/*
@brief
Reads from nRF24L01+ device in its own format.

@param
param[in] reg_address address that is wated to be read

Returns read value.
*/
uint8_t 
read_nrf_reg(const uint8_t reg_address)
{
    uint8_t read_buf;
    uint8_t temp_regaddress = (reg_address);
    cs_select();
    spi_write_blocking(spi0,&temp_regaddress,1);
    spi_read_blocking(spi0,0,&read_buf,1);
    cs_deselect();
    return read_buf;
}

/*
@brief
Adds the given bytes to tx fifo.

@param
param[in] buf Byte which will be writen to fifo.
*/
void 
write_tx_payload_byte(uint8_t buf)
{
    uint8_t write_tx_command_buf = WRITE_TX_FIFO;
    cs_select();
    spi_write_blocking(spi0,&write_tx_command_buf,1);
    spi_write_blocking(spi0,&buf,1);
    cs_deselect();
    busy_wait_us_32(10);
    ce_active();
    busy_wait_us_32(300);
    ce_deactive();
}

/*
@brief
Reads Rx payload and returns the byte.

Returns the read value.
*/
uint8_t 
read_rx_payload_byte()
{
    uint8_t read_rx_command_buf = READ_RX_FIFO;
    uint8_t read_buf=0;
    cs_select();
    spi_write_blocking(spi0,&read_rx_command_buf,1);
    spi_read_blocking(spi0,0,&read_buf,1);
    cs_deselect();
    return read_buf;
}

/*
@bried
Reads status registers


Returns status register
*/
uint8_t
read_status()
{
    uint8_t read_buffer;
    uint8_t nop_command_buf = NOP;
    cs_select();
    spi_write_read_blocking(spi0,&nop_command_buf,&read_buffer,sizeof(read_buffer));
    cs_deselect();
    return read_buffer;
}
/*
@brief 
Flushes TX FIFO on NRF device.
*/
void
flush_tx_nrf()
{
    uint8_t flush_tx_command_buf = FLUSH_TX;
    cs_select();
    spi_write_blocking(spi0,&flush_tx_command_buf,sizeof(uint8_t));
    cs_deselect();
}

/*
@brief
Flushes RX FIFO of NRF device.
*/
void
flush_rx_nrf()
{
    uint8_t flush_rx_command_buf = FLUSH_RX;
    cs_select();
    spi_write_blocking(spi0,&flush_rx_command_buf,sizeof(uint8_t));
    cs_deselect();
}

/*
@brief
Reads the status of fifo.

Returns status of fifo. uint8_t.
*/
uint8_t 
read_fifo_status()
{
    uint8_t status_addr_buf = FIFO_STATUS_ADDR;
    uint8_t status_buf;
    cs_select();
    spi_write_blocking(spi0,&status_addr_buf,1);
    spi_read_blocking(spi0,0,&status_buf,1);
    cs_deselect();
    return status_buf;
}