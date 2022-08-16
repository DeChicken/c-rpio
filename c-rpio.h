#ifndef PI_GPIO_H   // If PI_GPIO_H is not defined...
#define PI_GPIO_H   // define it (declare)

#include <inttypes.h>

// ---Constants--- //
// Addresses
#define BCM2711_PERI_BASE 0xFE000000				// Refer to Sec 1.2.4 in BCM2711 Datasheet
#define GPIO_BASE (BCM2711_PERI_BASE + 0x200000)	// Refer to Sec 5.2 in BCM2711 Datasheet

// Offsets
#define GPFSEL0 0x0		// GPFSEL0 offset from GPIO_BASE
#define GPFSEL1 0x4		// GPFSEL1 offset from GPIO_BASE
#define GPFSEL2 0x8		// GPFSEL2 offset from GPIO_BASE
#define GPSET0 0x1C		// GPSET0 offset from GPIO_BASE
#define GPCLR0 0x28		// GPCLR0 offset from GPIO_BASE

// Limits
#define MIN_PIN_INDEX 0		// Minimum GPIO pin index
#define MAX_PIN_INDEX 27	// Maxmimum GPIO pin index
#define GPIO_REG_MAX_OFFSET 0xF0	// Max register offset from GPIO_BASE

// Constants
#define GPFSEL_OPT_LEN 3	// GPFSEL registers option length (bits)
#define INPUT 0
#define OUTPUT 1
#define ALTFUNC0 4
#define ALTFUNC1 5
#define ALTFUNC2 6
#define ALTFUNC3 7
#define ALTFUNC4 3
#define ALTFUNC5 2
#define HIGH 1
#define LOW 0

// ---User Functions--- //
void gpio_setup(void);

void printlnBin32(uint32_t data);

void printGPIO(void);

int pinIsValid(int pin);

int pinMode(int pin, int mode);

int digitalWrite(int pin, int level);

int offsetIsValid(int offset);

uint32_t read_reg(int offset);

#endif              // End of the #ifndef PI_GPIO_H