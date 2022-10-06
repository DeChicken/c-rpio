#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include "c-rpio.h"

int main(void)
{
    int tx_pin = 0;
    int rx_pin = 1;
    
    cfg_uart(2);
}
