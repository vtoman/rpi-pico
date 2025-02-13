#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define I2C0_PORT i2c0 // Using I2C0
#define I2C0_SDA_PIN 20      
#define I2C0_SCL_PIN 21     

#define I2C1_PORT i2c1 // Using I2C1
#define I2C1_SDA_PIN 6      
#define I2C1_SCL_PIN 7      

void scan_i2c() {
    printf("\nScanning I2C0 Bus...\n");

    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        uint8_t dummy_data;
        int result = i2c_read_blocking(I2C0_PORT, addr, &dummy_data, 1, false);

        if (result >= 0) {
            printf("✅  Device found at address: 0x%02X\n", addr);
        } else {
            //printf("0x%02X ", addr);
        }
    }
    printf("I2C0 Scan Complete.\n");

    printf("\nScanning I2C1 Bus...\n");

    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        uint8_t dummy_data;
        int result = i2c_read_blocking(I2C1_PORT, addr, &dummy_data, 1, false);

        if (result >= 0) {
            printf("✅  Device found at address: 0x%02X\n", addr);
        } else {
            //printf("0x%02X ", addr);
        }
    }
    printf("I2C1 Scan Complete.\n");
}

void initializeI2C0() {
    i2c_init(I2C0_PORT, 100 * 1000); // Set speed to 100kHz
    gpio_set_function(I2C0_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C0_SDA_PIN);
    gpio_pull_up(I2C0_SCL_PIN);
}

void initializeI2C1() {
    i2c_init(I2C1_PORT, 100 * 1000); // Set speed to 100kHz
    gpio_set_function(I2C1_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C1_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C1_SDA_PIN);
    gpio_pull_up(I2C1_SCL_PIN);
}

int main() {
    stdio_init_all();  // Needed for printf to work
    sleep_ms(1000);    // Delay for USB serial connection

    initializeI2C0();
    initializeI2C1();

    while (true) {
        scan_i2c();
        sleep_ms(5000);  // Scan every 5 seconds
    }
}
