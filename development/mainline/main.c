#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include "c-rpio.h"
#include <termios.h>

int main(void)
{
    char *dev = "/dev/ttyAMA1";
    speed_t baud_rate;

    int UART = serial_begin(dev, baud_rate);

    int myInt = 0XF0F0F0F0;

    //serial_write(UART, &myInt, sizeof(myInt));

    if (serial_available(UART))
        printf("Data is ready\n");
    else
        printf("Data not ready\n");

}
