#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include "c-rpio.h"

int main(void)
{
    int offset = 0x4;
    int pin = 13;
    int value = 3;

    time_t delay = 0;
    long delayms = 500;

    int channel = 1;
    int PWEN = 1;
    int MODE = 0;
    int RPTL = 0;
    int SBIT = 0;
    int POLA = 0;
    int USEF = 0;
    int MSEN = 0;

    struct timespec delayts = {delay, delayms * 1000000};

    //printf("pwm_cfg returns %d\n", pwm_cfg(channel, PWEN, MODE, RPTL, SBIT, POLA, USEF, MSEN));
    //printf("pwm_cfg returns %d\n", pwm_cfg(channel + 1, PWEN, MODE, RPTL, SBIT, POLA, USEF, MSEN));
    //printf("analogWrite returns %d\n", analogWrite(pin, value));

    /*
    pinMode(13, OUTPUT);
    while (1 == 1)
    {
        digitalWrite(13, HIGH);
        nanosleep(&delayts, NULL);
        digitalWrite(13, LOW);
        nanosleep(&delayts, NULL);
    }
    */

    pinMode(pin, ALT0);
    analogWrite(pin, value);

    mini_main();
    
}
