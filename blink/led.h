#ifndef LED_H
#define LED_H

#include <stdbool.h>

int pico_led_init(void); 

void pico_set_led(bool led_on); 

#endif