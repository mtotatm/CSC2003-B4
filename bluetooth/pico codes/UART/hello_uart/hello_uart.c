/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include <time.h>

/// \tag::hello_uart[]

#define UART_ID uart0
#define BAUD_RATE 9600

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#define UART_TX_PIN 0
#define UART_RX_PIN 1


int main() {
    //absolute_time_t start_time = get_absolute_time();
    // absolute_time_t end_time;
    absolute_time_t end_time = make_timeout_time_us(1000000);
    int count = 0;
    // int execution_time;
    char output[50];
    // Set up our UART with the required speed.
    uart_init(UART_ID, BAUD_RATE);

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Use some the various UART functions to send out data
    // In a default system, printf will also output via the default UART

    // Send out a character without any conversions
    while(1){
        uart_puts(UART_ID, "A");
        count++;
        if(time_reached(end_time)){
            break;
        }
    }
    sprintf(output,"%d",count);
    uart_puts(UART_ID, output);
    
    // while(1){
    //     uart_puts(UART_ID, "A");
    //     count++;
    //     if (count == 994){
    //         end_time = get_absolute_time();
    //         execution_time = absolute_time_diff_us(start_time,end_time);
    //         break;
    //     }
    // }
    // sprintf(output,"%d",execution_time);
    // uart_puts(UART_ID, output);
    // Send out a character but do CR/LF conversions
    //uart_putc(UART_ID, 'B');

    // Send out a string, with CR/LF conversions
    //uart_puts(UART_ID, " Hello, UART!\n");
}

/// \end::hello_uart[]
