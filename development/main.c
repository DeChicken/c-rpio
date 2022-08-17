#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include "c-rpio.h"

int main(void)
{
    int pin = 27;
    int offset = 0;

    pinMode(pin, INPUT);
    printf("GPFSEL0 after pin %d set as input:\n", pin);
    printlnBin32(read_reg(offset));

    pinMode(pin, OUTPUT);
    printf("GPFSEL0 after pin %d set as output:\n", pin);
    printlnBin32(read_reg(offset));

    printf("Setting pin %d to HIGH...\n");
    digitalWrite(pin, HIGH);
    printf("digitalRead(pin) returns %d\n", digitalRead(pin));
    printf("Press any key to continue...");
    getchar();

    printf("Setting pin %d to LOW...\n");
    digitalWrite(pin, LOW);
    printf("digitalRead(pin) returns %d\n", digitalRead(pin));
    printf("Press any key to continue...");
    getchar();

}
