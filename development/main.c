#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include "c-rpio.h"

int main(void)
{
    int offset = 0x4;
    int pin = 12;
    int delay = 3;
    int value = 100;

    int channel = 1;
    int PWEN = 1;
    int MODE = 0;
    int RPTL = 0;
    int SBIT = 0;
    int POLA = 0;
    int USEF = 0;
    int MSEN = 0;

    //printf("pwm_cfg returns %d\n", pwm_cfg(channel, PWEN, MODE, RPTL, SBIT, POLA, USEF, MSEN));
    //printf("pwm_cfg returns %d\n", pwm_cfg(channel + 1, PWEN, MODE, RPTL, SBIT, POLA, USEF, MSEN));
    //printf("analogWrite returns %d\n", analogWrite(pin, value));

    /*
    pinMode(pin, OUTPUT);
    while (1 == 1)
    {
        digitalWrite(pin, HIGH);
        sleep(3);
        digitalWrite(pin, LOW);
        sleep(3);
    }
    */

    pinMode(pin, ALT0);
    pinMode(pin + 1, ALT0);
    analogWrite(pin, value);
    analogWrite(pin + 1, value);

}
