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

    cout << endl << "Test for serial output" << endl;
    printf("Counting: %d ", i);
    while (true) {
        // Flush is needed as endl or \n is not used.
        cout << ++i << " " << flush; 
        sleep_ms(1000);
    }
}
