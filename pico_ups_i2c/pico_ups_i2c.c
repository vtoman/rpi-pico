/**
 * ETA6003 - Recharger chip
 * GP6 -> SDA
 * GP7 -> SCL
 * 
 * Bus     Address
 * I2C1 -> 0x43
 * 
 * INA 219 I2C Registers
 * 00 RW  Configuration     Configure how INA219 operates
 * 01 R   Shunt voltage     Measure voltage drop accross a shunt resistor
 * 02 R   Bus voltage       Voltage on the bus (or the battery voltage)
 * 03 R   Power             Calculated power (bus voltage x current)
 * 04 R   Current           Provides calculated current flowing through the shunt
 * 05 RW  Calibration       Configure how INA219 scales its measurements
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// I2C configuration
#define I2C_PORT i2c1
#define I2C_BAUD 100000      // 100 kHz
#define I2C_ADDR 0x43        // INA219 I2C address

// INA219 Register Addresses
#define INA219_REG_CONFIG       0x00
#define INA219_REG_SHUNT_VOLTAGE 0x01
#define INA219_REG_BUS_VOLTAGE   0x02
#define INA219_REG_POWER         0x03
#define INA219_REG_CURRENT       0x04
#define INA219_REG_CALIBRATION   0x05

// I2C1 pins: using GP6 for SDA and GP7 for SCL
#define SDA_PIN 6
#define SCL_PIN 7

/**
 * Helper function to read a 16-bit register from the INA219.
 * Returns the 16-bit value, or -1 on error.
 */
int16_t read_ina219_register(uint8_t reg) {
    uint8_t buf[2];
    int ret;

    // Write the register address with no stop condition.
    ret = i2c_write_blocking(I2C_PORT, I2C_ADDR, &reg, 1, true);
    if (ret != 1) {
        printf("Error writing register 0x%02x\n", reg);
        return -1;
    }

    // Read 2 bytes from the register.
    ret = i2c_read_blocking(I2C_PORT, I2C_ADDR, buf, 2, false);
    if (ret != 2) {
        printf("Error reading register 0x%02x\n", reg);
        return -1;
    }

    // Combine the two bytes into a 16-bit value (big-endian).
    return (int16_t)((buf[0] << 8) | buf[1]);
}

void initializeI2C() {
    // Initialize I2C1 at the specified baud rate
    i2c_init(I2C_PORT, I2C_BAUD);
    // Configure the I2C pins
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
}

int main() {
    stdio_init_all();
    sleep_ms(2000);  // Allow time for USB serial connection to initialize

    initializeI2C();

    while (1) {
        int16_t shunt_voltage_raw = read_ina219_register(INA219_REG_SHUNT_VOLTAGE);
        int16_t bus_voltage_raw   = read_ina219_register(INA219_REG_BUS_VOLTAGE);
        int16_t power_raw         = read_ina219_register(INA219_REG_POWER);
        int16_t current_raw       = read_ina219_register(INA219_REG_CURRENT);

        // Convert shunt voltage:
        // The INA219 shunt voltage register has an LSB of 10µV,
        // so we multiply by 0.01 to convert to mV.
        float shunt_voltage_mV = shunt_voltage_raw * 0.01f;

        // Convert bus voltage:
        // Bits [15:3] contain the bus voltage (LSB = 4 mV).
        // Thus, bus voltage in mV = (bus_voltage_raw >> 3) * 4.
        int16_t bus_voltage_mV = ((uint16_t)bus_voltage_raw >> 3) * 4;

        // For current and power, the conversion factors depend on the calibration
        // settings you use (typically set in the calibration register).
        // Here we simply print the raw register values.
        printf("Shunt Voltage: %.2f mV, Bus Voltage: %d mV, Power: %d, Current: %d\n",
               shunt_voltage_mV, bus_voltage_mV, power_raw, current_raw);

        sleep_ms(1000);
    }
    return 0;
}
