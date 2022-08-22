// c-rpio.c and c-rpio.h must be in the same directory before building!!!

#include <unistd.h>
#include <time.h>
#include "c-rpio.h"

int main() 
{
    // pins 12, 13, 18, and 19 have hardware PWM functionality\
    // You must set which pins you plan to use in config.txt of the pi
    // See https://github.com/dotnet/iot/blob/main/Documentation/raspi-pwm.md#enabling-hardware-pwm
    int pin = 13;
    time_t delay = 0;
    long delayms = 5;

    struct timespec delayts = {delay, delayms * 1000000};

    // See the BCM datasheet GPIO section to see the mode to set each pin to for PWM
    //     For the BCM2711 (Raspberry Pi 4 Model B):
    //     PWM mode for pin 12 and 13 is ALT0  
    //     PWM mode for pin 18 and 19 is ALT5

    pinMode(pin, ALT0);
    while (1 == 1)
    {
        for (int i = 0; i < 257; i++)
        {
            analogWrite(pin, i);
            nanosleep(&delayts, NULL);
        }
        for (int i = 256; i >= 0; i--)
        {
            analogWrite(pin, i);
            nanosleep(&delayts, NULL);
        }
    }
}
