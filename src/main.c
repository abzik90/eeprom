#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <stdio.h>

#define I2C_SPEED 100000 // 100 kHz
#define I2C_DEVICE_ADDR 0x50 // Your EEPROM device address on i2c bus. You can detect it by using i2c-tools
#define I2C_TIMEOUT 1000 // How many times your code will check flags

// You can change I2C bus or pins; Don't forget to change AF as well
void i2c_setup(void) {
    // Enable I2C clock and GPIO clock
    rcc_periph_clock_enable(RCC_I2C1);
    rcc_periph_clock_enable(RCC_GPIOB);

    // Setup GPIOs
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO8 | GPIO9);
	gpio_set_af(GPIOB, GPIO_AF4, GPIO8 | GPIO9);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, GPIO8|GPIO9);
	i2c_peripheral_disable(I2C1);

    // Setup I2C
    i2c_set_standard_mode(I2C1);
    i2c_set_speed(I2C1, i2c_speed_sm_100k, rcc_apb1_frequency / 1000000);
    i2c_set_dutycycle(I2C1, I2C_CCR_DUTY_DIV2);
    i2c_enable_ack(I2C1);
    i2c_peripheral_enable(I2C1);
}

// Returns true, if I2C bus is busy
bool i2c_busy(uint32_t i2c, const uint16_t timeout){
    uint16_t b = 0;
    for(b = 0;(I2C_SR2(i2c) & I2C_SR2_BUSY) && (b < timeout); ++b);
    if (b >= timeout){
        i2c_send_stop(i2c);
        return true;
    }
    return false;
}

// Returns true, if unable to switch to master mode
bool i2c_master_mode(uint32_t i2c, const uint16_t timeout){
    uint16_t b = 0;
    for (b = 0; (!((I2C_SR1(i2c) & I2C_SR1_SB) & (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY)))) && (b<timeout); ++b);
    if (b >= timeout){
        i2c_send_stop(i2c);
        return true;
    }
    return false;
}

// Returns true, if unable to send the device address
bool i2c_address_wait(uint32_t i2c, const uint16_t timeout){
    uint16_t b = 0;
    for (b = 0; !(I2C_SR1(i2c) & I2C_SR1_ADDR)&& (b < timeout); ++b);
    if (b >= timeout){
        i2c_send_stop(i2c);
        return true;
    }
    return false;
}
// Returns true, if unable to send data to EEPROM
bool i2c_data_wait(uint32_t i2c, const uint16_t timeout){
    uint16_t b = 0;
    for (b = 0; (!(I2C_SR1(i2c) & (I2C_SR1_BTF))) && (b < I2C_TIMEOUT); ++b);
    if (b >= timeout){
        i2c_send_stop(i2c);
        return true;
    }
    return false;
}
// Returns true, if unable to receive data from EEPROM
bool i2c_received_wait(uint32_t i2c, const uint16_t timeout){
    uint16_t b = 0;
    for(b = 0;(!(I2C_SR1(i2c) & (I2C_SR1_RxNE))) && (b < I2C_TIMEOUT); ++b);
    if (b >= timeout){
        i2c_send_stop(i2c);
        return true;
    }
    return false;
}

/*
i2c (uint32_t): I2C Bus name - I2C1, I2C2
device_addr (uint8_t): Address of the device - 0x50
register_addr (uint8_t): Address to change - 0x11
data (uint8_t): data to be sent
*/
static void eeprom_write(uint32_t i2c, uint8_t device_addr, uint8_t register_addr, uint8_t data)
{
    
    if(i2c_busy(i2c, I2C_TIMEOUT)) return; // Wait for i2c bus or return on timeout
    i2c_send_start(i2c);

    if(i2c_master_mode(i2c, I2C_TIMEOUT)) return; // Wait for master mode selected or return on timeout
    i2c_send_7bit_address(i2c, device_addr, I2C_WRITE);

    if(i2c_address_wait(i2c, I2C_TIMEOUT)) return; // Waiting for address is transferred till timeout.
    (void)I2C_SR2(i2c); // Clearing ADDR condition sequence.

    i2c_send_data(i2c, register_addr);
    if(i2c_data_wait(i2c, I2C_TIMEOUT)) return; // Waiting for register/data to be transferred.   

    i2c_send_data(i2c, data);
    if(i2c_data_wait(i2c, I2C_TIMEOUT)) return;

    i2c_send_stop(i2c);
}

/*
i2c (uint32_t): I2C Bus name - I2C1, I2C2
device_addr (uint8_t): Address of the device - 0x50
register_addr (uint8_t): Address to change - 0x11
data(uint8_t *): received data will be written here
*/
bool eeprom_read(uint32_t i2c, uint8_t device_addr, uint8_t register_addr, uint8_t *data) {
    if(i2c_busy(i2c, I2C_TIMEOUT)) return false; // Wait for i2c bus or return on timeout
    i2c_send_start(i2c);

    if(i2c_master_mode(i2c, I2C_TIMEOUT)) return false; // Wait for master mode selected or return on timeout
    i2c_send_7bit_address(i2c, device_addr, I2C_WRITE); // Select device address for further RW operations

    if(i2c_address_wait(i2c, I2C_TIMEOUT)) return false; // Waiting for address to be transferred till timeout.
    (void)I2C_SR2(i2c); // Clear ADDR condition sequence

    i2c_send_data(i2c, register_addr); // Send register address to read from
    if(i2c_data_wait(i2c, I2C_TIMEOUT)) return false; // Waiting for register/data to be transferred.   

    // Now listening device for its response
    i2c_send_start(i2c);
    if(i2c_master_mode(i2c, I2C_TIMEOUT)) return false; // Wait for master mode selected or return on timeout
    
    i2c_send_7bit_address(i2c, device_addr, I2C_READ); // Send device address for read operation
    if(i2c_address_wait(i2c, I2C_TIMEOUT)) return false; // Waiting for address is transferred till timeout.
    (void)I2C_SR2(i2c); // Clearing ADDR condition sequence.
    
    (void)i2c_get_data(i2c); // Perform dummy read to initiate data reception
    if(i2c_received_wait(i2c, I2C_TIMEOUT)) return false; // Wait for byte received till timeout
    *data = i2c_get_data(i2c); // Read data from the buffer

    i2c_send_stop(i2c);
    return true;
}



int main(void) {
    rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_84MHZ]);
    i2c_setup();

    // Write to the device
    // We're writing to the device 0x50 on I2C1 value of 0x03 on the register 0xCD
    eeprom_write(I2C1, I2C_DEVICE_ADDR, 0xCD, 0x03);
   
    // Delay between write and read
    int t = 500000;
	while(t-- > 0) __asm__("nop");

    // Read from the device
    // We're reading data from the register 0xCD
    uint8_t received_data = 0;
    bool received = eeprom_read(I2C1, I2C_DEVICE_ADDR, 0xCD, &received_data);
    
    while (1);
    return 0;
}
