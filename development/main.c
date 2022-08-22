#include <stdio.h>
#include <math.h>
#include "c-rpio.h"

int main(void)
{
    int pin = 13;
    int value = 0;
    int perc = 0;

    // Get desired percentage
    while (1 == 1)
    {
        printf("Enter duty cycle as a percentage: ");
        scanf("%d", &perc);

        value = (int)floor(((double)perc / 100.0) * (double)PWM_DEFAULT_RANGE);

        printf("value = %d\n", value);

        if (value >= 0 && value <= 256)
        {
            printf("Value to write: %d\n", value);
            analogWrite(pin, value);
        }
        else
            printf("Value of %d is invalid\n");
    }

}