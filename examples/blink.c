// c-rpio.c and c-rpio.h must be in the same directory before building!!!

#include <unistd.h>
#include "c-rpio.h"

int main()
{
    int pin = 1;
    int delay = 1;

    pinMode(pin, OUTPUT);
    
    while (1 == 1)
    {
        digitalWrite(pin, HIGH);
        sleep(delay);
        digitalWrite(pin, LOW);
        sleep(delay);
    }
}
