#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include "c-rpio.h"

int main(void)
{
    int tx_pin = 0;
    int rx_pin = 1;
    
    // Setup pins for UART2
    pinMode(tx_pin, ALT4);
    pinMode(rx_pin, ALT4);

    // Open file descriptor to UART2
    FILE *uart2 = fopen("/dev/ttyAMA2", "rw");

    fputc('o', uart2);

    // Close the file
    fclose(uart2);
}
