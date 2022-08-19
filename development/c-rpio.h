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
#define GPLEV0 0x34		// GPLEV0 offset from GPIO_BASE

// Limits
#define MIN_PIN_INDEX 0				// Minimum GPIO pin index
#define MAX_PIN_INDEX 27			// Maxmimum GPIO pin index
#define GPIO_REG_MAX_OFFSET 0xF0	// Max register offset from GPIO_BASE

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

// ---User Functions--- //
volatile void *mem_setup(off_t base_addr, size_t length);

void printlnBin32(uint32_t data);

void printGPIO(void);

int pinIsValid(int pin);

int pinMode(int pin, int mode);

int digitalWrite(int pin, int level);

int offsetIsValid(int offset);

int digitalRead(int pin);

uint32_t read_reg(volatile void **mempp, int offset);

int edit_reg_bits(volatile void **mempp, int offset, uint32_t bits, int length, int index);

int pwm_cfg(int channel, int PWEN, int MODE, int RPTL, int SBIT, int POLA, int USEF, int MSEN);

int analogWrite(int pin, int value);

#endif              // End of the #ifndef PI_GPIO_H