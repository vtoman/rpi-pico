/**
 *  Test for Raspberry Pi Pico board led light.
 */

#include "pico/stdlib.h"
#include "led.h"

#define LED_DELAY_MS 1000

int main()
{
    pico_led_init();
    while (true)
    {
        pico_set_led(true);
        sleep_ms(LED_DELAY_MS);
        pico_set_led(false);
        sleep_ms(LED_DELAY_MS);
    }
}
