/**
 *  RPI Pico 2W board led light.
 */

 #include "pico/stdlib.h"
 #include "pico/cyw43_arch.h"
 
 int main() {
     stdio_init_all();
     if (cyw43_arch_init()) {
         printf("Wi-Fi init failed");
         return -1;
     }
     while (true) {
        printf("The blink has blinked!\n");
         cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
         sleep_ms(10);
         cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
         sleep_ms(10000);
     }
 }