// uart_loopback.c
/*
    SUMMARY
        This program writes data to UART2 on the Raspberry Pi 4B.
        The baud rate is 9600. In order for this to run properly,
        GPIO0 must be connected to GPIO1 and the UART2 device tree
        overlay must be enabled in /boot/config.txt. Please read
        the warning below.
    
    WARNING
        Do not connect GPIO0 to GPIO1 until you know that those pins
        are in TX and RX mode, otherwise you might make a short. You
        can check the modes of the GPIO pins by typing the following
        command:
            
            $ raspi-gpio get 0-27
        
        For this program you should see these two lines in the output:

            GPIO 0: level=1 fsel=3 alt=4 func=TXD2 pull=NONE
            GPIO 1: level=1 fsel=3 alt=4 func=RXD2 pull=UP

        Once you see these two lines, it is safe to connect GPIO0 to
        GPIO1.

*/


#include <stdio.h>      // printf and friends
#include <termios.h>    // speed_t and baud rate constants defined here
#include "c-rpio.h"     // Raspberry Pi 4B peripheral functions

#define BUFFER_SIZE 64  // Input buffer will be 64 bytes long

int main(void)
{
    int tx_pin = 0;             // GPIO0 is TX pin for UART2
    int rx_pin = 1;             // GPIO1 is RX pin for UART2

    pinMode(tx_pin, ALT4);      // ALT4 for GPIO0 is TXD2
    pinMode(rx_pin, ALT4);      // ALT4 for GPIO1 is RXD2

    /*
        The following lines were added to /boot/config.txt:
            enable_uart=1
            dtoverlay=uart2
    */

    char *uart_dev = "/dev/ttyAMA1";     // Path to the UART device
    
    // Note: You may need to try other ttyAMA# devices. It does not
    //     necessarily correlate to the UART number.
    
    speed_t baud_rate = B9600;      // 9600 baud

    // Set up the serial port.
    // serial_begin returns a file descriptor
    int uart_fd = serial_begin(uart_dev, baud_rate);

    // Define the data to write
    char data[] = "Hello, world!";

    // Notify user of data length
    printf("Data is %d bytes long\n", sizeof(data));

    // Write the data to the file descriptor
    serial_write(uart_fd, data, sizeof(data));

    // Wait to hear it back
    int timeout = -1;   // No timeout

    if (serial_available(uart_fd, timeout))
        printf("Data is ready\n");
    else
        printf("Data not ready\n");
    
    // Initialize the input buffer
    char input_buf[BUFFER_SIZE];
    serial_read(uart_fd, &input_buf, sizeof(input_buf));
    printf("Data returned is: \"%s\"\n", input_buf);

    // Close the file descriptor
    serial_close(uart_fd);
}
