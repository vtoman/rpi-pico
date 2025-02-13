#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <math.h>

#define I2C_PORT i2c0
#define TSL2591_ADDR 0x29    // Update to 0x29 if needed per your sensor's datasheet
#define TSL2591_COMMAND 0xA0 // Command bit for register access
#define TSL2591_ID_REG 0x12  // ID register address

#define I2C0_SDA_PIN 20
#define I2C0_SCL_PIN 21

#define SDA_PIN I2C0_SDA_PIN
#define SLC_PIN I2C0_SCL_PIN

// TSL2591 register addresses
// TSL2591 register addresses
#define TSL2591_ENABLE_REG      0x00
#define TSL2591_CONTROL_REG     0x01
#define TSL2591_DEVICE_ID_REG   0x12
#define TSL2591_CHAN0_LOW_REG   0x14
#define TSL2591_CHAN0_HIGH_REG  0x15
#define TSL2591_CHAN1_LOW_REG   0x16
#define TSL2591_CHAN1_HIGH_REG  0x17

// Enable register values: bit 0 = power on, bit 1 = ALS (Ambient Light Sensing) enable
#define TSL2591_ENABLE_POWERON 0x01
#define TSL2591_ENABLE_AEN 0x02
#define TSL2591_ENABLE_VALUE (TSL2591_ENABLE_POWERON | TSL2591_ENABLE_AEN) // 0x03

// Control register: lower nibble for integration time, upper nibble for gain.
// For example, 0x00 might correspond to 100 ms integration time and low gain.
// (Consult your sensor’s datasheet or library for available options.)
// #define TSL2591_CONTROL_VALUE   0x13 // Medium gain, integration 400ms
#define TSL2591_CONTROL_VALUE   0x15 // Maximum gain, integration 600ms

void initializeI2C()
{
    // Set up I2C at 100 kHz.
    i2c_init(I2C_PORT, 100 * 1000);

    // Configure I2C pins.
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SLC_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SLC_PIN);
}

void cmdReadID()
{
    // Build the command byte to read the sensor's ID register.
    uint8_t reg = TSL2591_COMMAND | TSL2591_ID_REG; // 0xA0 | 0x12 = 0xB2

    // Write the command (register address) to the sensor.
    int ret = i2c_write_blocking(I2C_PORT, TSL2591_ADDR, &reg, 1, true);
    if (ret != 1)
    {
        printf("I2C write failed!\n");
    }
    else
    {
        // Read one byte from the sensor (the ID).
        uint8_t id;
        ret = i2c_read_blocking(I2C_PORT, TSL2591_ADDR, &id, 1, false);
        if (ret != 1)
        {
            printf("I2C read failed!\n");
        }
        else
        {
            printf("TSL2591 ID: 0x%02X\n", id);
        }
    }
}

void cmdEnableSensor()
{
    // --- Enable the sensor ---
    // Write to the ENABLE register to power on the device and enable ALS
    uint8_t cmd = TSL2591_COMMAND | TSL2591_ENABLE_REG;
    uint8_t enable_val = TSL2591_ENABLE_VALUE; // 0x03
    int ret = i2c_write_blocking(I2C_PORT, TSL2591_ADDR, &cmd, 1, true);
    if (ret != 1)
    {
        printf("I2C write (enable cmd) failed!\n");
    }
    ret = i2c_write_blocking(I2C_PORT, TSL2591_ADDR, &enable_val, 1, false);
    if (ret != 1)
    {
        printf("I2C write (enable value) failed!\n");
    }
}

void readSensorChannels()
{
    uint8_t reg = TSL2591_COMMAND | 0x20 | TSL2591_CHAN0_LOW_REG;
    uint8_t data[4] = {0};
    int ret = i2c_write_blocking(I2C_PORT, TSL2591_ADDR, &reg, 1, true);
    if (ret != 1)
    {
        printf("I2C write (data command) failed!\n");
    }
    else
    {
        ret = i2c_read_blocking(I2C_PORT, TSL2591_ADDR, data, 4, false);
        if (ret != 4)
        {
            printf("I2C read (channel data) failed!\n");
        }
        else
        {
            // Combine low and high bytes for each channel (little-endian)
            uint16_t ch0 = ((uint16_t)data[1] << 8) | data[0];
            uint16_t ch1 = ((uint16_t)data[3] << 8) | data[2];

            printf("CH0: %u, CH1: %u, Lux: %.2f\n", ch0, ch1);
        }
    }
}

void setIntegrationTimeAndGain() {
     // Write to the CONTROL register to set integration time and gain
    uint8_t cmd = TSL2591_COMMAND | TSL2591_CONTROL_REG;
    uint8_t control_val = TSL2591_CONTROL_VALUE;  // default: 100 ms integration, low gain
    int ret = i2c_write_blocking(I2C_PORT, TSL2591_ADDR, &cmd, 1, true);
    if (ret != 1) {
        printf("I2C write (control cmd) failed!\n");
    }
    ret = i2c_write_blocking(I2C_PORT, TSL2591_ADDR, &control_val, 1, false);
    if (ret != 1) {
        printf("I2C write (control value) failed!\n");
    }
}

int main()
{
    stdio_init_all();

    initializeI2C();
    sleep_ms(1000);

    cmdEnableSensor();
    sleep_ms(1000);

    setIntegrationTimeAndGain();
    sleep_ms(1000);

    cmdReadID();

    while (true)
    {        
        readSensorChannels();        
        sleep_ms(1000);
    }

    return 0;
}
