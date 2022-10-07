#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include "c-rpio.h"
#include <termios.h>

int main(void)
{
    char *dev = "/dev/ttyAMA1";
    speed_t baud_rate = B4800;

    int UART = serial_begin(dev, baud_rate);

    char c[] = "Glizzy";

    printf("Glizzy is %d bytes long\n", sizeof(c));

    serial_write(UART, c, sizeof(c));

    if (serial_available(UART))
        printf("Data is ready\n");
    else
        printf("Data not ready\n");
    
    char res[10] = "";
    serial_read(UART, res, sizeof(c));
    printf("res is %s\n", res);

    serial_close(UART);
}
