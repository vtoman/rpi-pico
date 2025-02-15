/**
 *  RPI Pico 2W board led light.
 */

 #include "pico/stdlib.h"
 #include "pico/cyw43_arch.h"
 
void on(uint32_t timeout) {
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
         sleep_ms(timeout);
}

void off(uint32_t timeout) {
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    sleep_ms(timeout);
}

 int main() {
     stdio_init_all();
     if (cyw43_arch_init()) {
         printf("Wi-Fi init failed");
         return -1;
     }
     while (true) {
        //printf("The blink has blinked!\n");
        on(40);
        off(200);
        on(40);
        off(10000);
     }
 }