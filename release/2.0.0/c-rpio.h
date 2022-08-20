#ifndef PI_GPIO_H   // If PI_GPIO_H is not defined...
#define PI_GPIO_H   // define it (declare)

#include <inttypes.h>

// Addresses
#define BCM2711_PERI_BASE 0xFE000000				// Refer to Sec 1.2.4 in BCM2711 Datasheet
#define GPIO_BASE (BCM2711_PERI_BASE + 0x200000)	// Refer to Sec 5.2 in BCM2711 Datasheet
#define PWM_BASE (BCM2711_PERI_BASE + 0X20C000)		// Refer to Sec 8.6 in BCM2711 Datasheet

// Offsets
#define GPFSEL0 0x0		// GPFSEL0 offset from GPIO_BASE
#define GPFSEL1 0x4		// GPFSEL1 offset from GPIO_BASE
#define GPFSEL2 0x8		// GPFSEL2 offset from GPIO_BASE
#define GPSET0 0x1C		// GPSET0 offset from GPIO_BASE
#define GPCLR0 0x28		// GPCLR0 offset from GPIO_BASE
#define GPLEV0 0x34		// GPLEV0 offset from GPIO_BASE
#define CTL 0x0			// CTL offset from PWM_BASE
#define STA 0x4			// STA offset from PWM_BASE
#define RNG1 0x10		// RNG1 offset from PWM_BASE
#define RNG2 0x20		// RNG2 offset from PWM_BASE
#define DAT1 0x14		// DAT1 offset from PWM_BASE
#define DAT2 0x24		// DAT2 offset from PWM_BASE
#define FIF1 0x18		// FIF1 offset from PWM_BASE

// Limits
#define MIN_PIN_INDEX 0				// Minimum GPIO pin index
#define MAX_PIN_INDEX 27			// Maxmimum GPIO pin index
#define GPIO_REG_MAX_OFFSET 0xF0	// Max register offset from GPIO_BASE
#define PWM_REG_MAX_OFFSET 0x24		// Max register offset from PWM_BASE

// Constants
#define GPFSEL_OPT_LEN 3	// GPFSEL registers option length (bits)
#define INPUT 0
#define OUTPUT 1
#define ALT0 4
#define ALT1 5
#define ALT2 6
#define ALT3 7
#define ALT4 3
#define ALT5 2
#define HIGH 1
#define LOW 0
#define PWM_DEFAULT_RANGE 256
#define REG_LEN_BITS 32

// ---User Functions--- //

void printlnBin32(uint32_t data);

int pinIsValid(int pin);

int pinMode(int pin, int mode);

int digitalWrite(int pin, int level);

int digitalRead(int pin);

int analogWrite(int pin, int value);

void printVersion(void);

#endif              // End of the #ifndef PI_GPIO_H