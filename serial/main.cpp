/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * 
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include <iostream>

using namespace std;

int main() {
    stdio_init_all();
    sleep_ms(4000); // wait to start

    int i = 0;

    std::cout << "Test for serial output\n";
    printf("Counting: %d ", i);
    while (true) {
        cout << ++i << " "; 
        sleep_ms(1000);
    }
}
