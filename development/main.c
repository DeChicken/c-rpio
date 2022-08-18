#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include "c-rpio_simp.h"

int main(void)
{
    int offset = 0x0;

    volatile void *gpptr = gpio_setup();

    for (int i = 0; i < 10; i++)
    {
        pinMode(i, INPUT);
        printlnBin32(read_reg(&gpptr, 0x0));
        pinMode(i, ALTFUNC3);
        printlnBin32(read_reg(&gpptr, 0x0));
        printf("\n");
    }
}
