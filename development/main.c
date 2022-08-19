#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include "c-rpio_simp.h"

int main(void)
{
    int offset = 0x4;
    int pin = 10;
    int delay = 3;

    volatile void *gpptr = gpio_setup();

    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);

    while (1 == 1)
    {
        sleep(delay);
        digitalWrite(pin, HIGH);
        printf("Pin %d level is %d\n", pin, digitalRead(pin));
        sleep(delay);
        digitalWrite(pin, LOW);
        printf("Pin %d level is %d\n", pin, digitalRead(pin));
    
    }

}
