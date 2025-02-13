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
#define INA219_REG_CONFIG        0x00
#define INA219_REG_SHUNT_VOLTAGE 0x01
#define INA219_REG_BUS_VOLTAGE   0x02
#define INA219_REG_POWER         0x03
#define INA219_REG_CURRENT       0x04
#define INA219_REG_CALIBRATION   0x05

// Example config value for 32V range, gain /8, 12-bit resolution continuous
// There are many valid configs; this one is fairly standard.
#define INA219_CONFIG_32V_8_GAIN_12BIT  0x3C1F

// Our chosen calibration for a 10 mΩ shunt, ~3A max current
// CAL = 40960 = 0xA000
#define INA219_CALIB_10mOHM_3A  40960

// Conversion factors based on the chosen calibration:
// Current LSB = 0.0001 A (0.1 mA)
// Power LSB   = 20 x Current LSB = 0.002 W (2 mW)
#define INA219_CURRENT_LSB  0.0001f // Amps per bit
#define INA219_POWER_LSB    0.002f  // Watts per bit

// I2C1 pins: using GP6 for SDA and GP7 for SCL
#define SDA_PIN 6
#define SCL_PIN 7

static int write_ina219_register_16(uint8_t reg, uint16_t value) {
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = (value >> 8) & 0xFF;
    buf[2] = value & 0xFF;
    int ret = i2c_write_blocking(I2C_PORT, I2C_ADDR, buf, 3, false);
    return (ret == 3) ? 0 : -1;
}

/**
 * Helper function to read a 16-bit register from the INA219.
 * Returns the 16-bit value, or -1 on error.
 */
static int16_t read_ina219_register_16(uint8_t reg) {
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

static void ina219_configure_10mOhm_3A(void) {
    // 1. Write the config register
    //    (Example: 32V range, gain /8, 12-bit continuous)
    if (write_ina219_register_16(INA219_REG_CONFIG, INA219_CONFIG_32V_8_GAIN_12BIT) < 0) {
        printf("Failed to write config register\n");
        return;
    }

    // 2. Write the calibration register
    if (write_ina219_register_16(INA219_REG_CALIBRATION, INA219_CALIB_10mOHM_3A) < 0) {
        printf("Failed to write calibration register\n");
        return;
    }
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

    // Configure INA219 for 10 mΩ, ~3A range
    ina219_configure_10mOhm_3A();

    while (1) {
        // Read raw registers
        int16_t shunt_voltage_raw = read_ina219_register_16(INA219_REG_SHUNT_VOLTAGE);
        int16_t bus_voltage_raw   = read_ina219_register_16(INA219_REG_BUS_VOLTAGE);
        int16_t power_raw         = read_ina219_register_16(INA219_REG_POWER);
        int16_t current_raw       = read_ina219_register_16(INA219_REG_CURRENT);

        // Convert shunt voltage:
        // The INA219 shunt voltage register has an LSB of 10µV,
        // so we multiply raw by 0.01 to convert to mV.
        float shunt_voltage_mV = shunt_voltage_raw * 0.01f;

        // Convert bus voltage:
        // Bits [15:3] contain the bus voltage (LSB = 4 mV).
        // So bus voltage in mV = (bus_voltage_raw >> 3) * 4.
        int16_t bus_voltage_mV = ((uint16_t)bus_voltage_raw >> 3) * 4;

        // Convert current using the chosen LSB
        // current (A) = current_raw * 0.0001
        float current_A = current_raw * INA219_CURRENT_LSB;

        // Convert power using the chosen LSB
        // power (W) = power_raw * 0.002
        float power_W = power_raw * INA219_POWER_LSB;

        printf("Shunt: %.2f mV, Bus: %d mV, Current: %.3f A, Power: %.3f W\n",
               shunt_voltage_mV, bus_voltage_mV, current_A, power_W);

        sleep_ms(1000);
    }
    return 0;
}
