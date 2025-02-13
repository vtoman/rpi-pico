#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <time.h>
#include <string.h>

#define I2C_PORT i2c1  // Using I2C1
#define SDA_PIN 6      // Correct SDA for I2C1
#define SCL_PIN 7      // Correct SCL for I2C1
#define I2C_ADDR 0x70 // Device temphum
#define SHTC3_ADDR 0x70  // SHTC3 I2C Address

void print_current_time() {
    time_t now = time(NULL);  // Get current system time
    struct tm *t = localtime(&now);  // Convert to local time structure

    printf("⏰  Current Time: %04d-%02d-%02d %02d:%02d:%02d\n",
           t->tm_year + 1900,  // Year (tm_year is years since 1900)
           t->tm_mon + 1,      // Month (tm_mon is 0-based)
           t->tm_mday,         // Day
           t->tm_hour,         // Hour
           t->tm_min,          // Minutes
           t->tm_sec);         // Seconds
}

// Function to check CRC (SHTC3 uses CRC-8)
bool check_crc(uint16_t data, uint8_t crc) {
    uint8_t computed_crc = 0xFF;  // Initial value
    uint8_t poly = 0x31;          // Polynomial (0x31 = x^8 + x^5 + x^4 + 1)

    computed_crc ^= (data >> 8);  // First byte
    for (int i = 0; i < 8; i++) {
        if (computed_crc & 0x80) computed_crc = (computed_crc << 1) ^ poly;
        else computed_crc <<= 1;
    }

    computed_crc ^= (data & 0xFF);  // Second byte
    for (int i = 0; i < 8; i++) {
        if (computed_crc & 0x80) computed_crc = (computed_crc << 1) ^ poly;
        else computed_crc <<= 1;
    }

    return (computed_crc == crc);
}

// Function to process SHTC3 measurement data
void process_shtc3_data(uint8_t data[6]) {
    uint16_t raw_temp = (data[0] << 8) | data[1];
    uint8_t temp_crc = data[2];

    uint16_t raw_humidity = (data[3] << 8) | data[4];
    uint8_t humidity_crc = data[5];

    // Validate CRC
    if (!check_crc(raw_temp, temp_crc)) {
        printf("❌  Temperature CRC check failed!\n");
        return;
    }
    if (!check_crc(raw_humidity, humidity_crc)) {
        printf("❌  Humidity CRC check failed!\n");
        return;
    }

    // Convert raw values to real-world values (per SHTC3 datasheet)
    float temperature = -45.0f + 175.0f * ((float)raw_temp / 65535.0f);
    float humidity = 100.0f * ((float)raw_humidity / 65535.0f);

    print_current_time();
    printf("🌡️  Temperature: %.2f°C\n", temperature);
    printf("💧  Humidity: %.2f%%\n", humidity);
}


bool check_i2c(uint8_t addr) {
    bool result = false;

    uint8_t dummy_data;
    int bytesRead = i2c_read_blocking(I2C_PORT, addr, &dummy_data, 1, false);

    if (bytesRead >= 0) {
        printf("Device found at address: 0x%02X\n", addr);
        return true;
    }

    return result;
}

void scan_i2c() {
    printf("\nScanning I2C Bus...\n");

    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        check_i2c(addr);
    }

    printf("I2C Scan Complete.\n");
}

void init_i2c() {
    i2c_init(I2C_PORT, 100 * 1000); // I2C 100kHz
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
}

void read_temphum() {
    // ---- Write 2 bytes: 0x78, 0x66 ----
    uint8_t write_data[2] = {0x78, 0x66};
    int write_result = i2c_write_blocking(I2C_PORT, I2C_ADDR, write_data, 2, false);
    
    if (write_result < 0) {
        printf("❌  I2C write failed!\n");
    } else {
        //printf("✅  Wrote 2 bytes to 0x70: 0x78 0x66\n");
    }

    // ---- Wait 14ms ----
    sleep_ms(14);

    // ---- Read 6 bytes from 0x70 ----
    uint8_t read_data[6] = {0}; // Buffer for incoming data
    int read_result = i2c_read_blocking(I2C_PORT, I2C_ADDR, read_data, 6, false);

    if (read_result < 0) {
        printf("❌  I2C read failed!\n");
    } else {
        process_shtc3_data(read_data);
    }
}

int main() {
    stdio_init_all(); 
    sleep_ms(6000);    
    init_i2c();
    scan_i2c();

    while (true) {
        read_temphum();
        sleep_ms(10000); 
    }
}
