/*
	Purpose:
		WIP: This program drives a gpio pin on the raspberry pi
	Author: David Craig
*/

#include <stdio.h>
#include <inttypes.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>

// Version
#define C_RPIO_VERSION "1.1.0"

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
#define ALTFUNC0 4
#define ALTFUNC1 5
#define ALTFUNC2 6
#define ALTFUNC3 7
#define ALTFUNC4 3
#define ALTFUNC5 2
#define HIGH 1
#define LOW 0

// PHILOSOPHY
// Make any functions a user might touch easy to debug

// Allowed PWM pins (I believe these are the same across all Raspberry Pi models?)
int pwm_pins[4] = {12, 13, 18, 19};

// Pointer to the gpio physical memory, null until gpio_setup() is run
static volatile void *gpio_ptr = NULL;

// Pointer to the pwm physical memory, null until pwm_setup() is run
static volatile void *pwm_ptr = NULL;

int offsetIsValid(int offset)
{
	// Check that offset is valid
	if (offset % 4 != 0)	// Offset not aligned
		return 0;
	
	if ((offset < 0) || (offset > GPIO_REG_MAX_OFFSET))	// Offset out of range
		return 0;
	
	// Passed the checks
	return 1;
}

int pinIsValid(int pin)
{
	if (pin < MIN_PIN_INDEX || pin > MAX_PIN_INDEX)
		return 0;
	else
		return 1;
}

volatile void *gpio_setup(void)
{	
	int memfd;
	volatile void *result_ptr;

	// Attempt to open /dev/mem
	if ((memfd = open("/dev/mem", O_RDWR | O_SYNC)) > 0) // Read only for now...
	/*
		O_SYNC description from the open(2) man page:
			"O_SYNC would also always flush the last
				modification timestamp metadata"
	*/
	{
		
		// Create mapping to the gpio registers
		result_ptr = mmap(
			NULL,					// Addr in virtual mem to map to
			GPIO_REG_MAX_OFFSET,	// Length of mapping
			PROT_READ | PROT_WRITE,	// Enable reading only, for now...
			MAP_SHARED, 			// Share map space with other processes
			memfd,					// File (device) to map from
			GPIO_BASE 				// Offset to peripheral base
		);
		close(memfd);	// Close stream with /dev/mem
        return result_ptr;
	}
	else
	{
		printf("[ERROR] gpio_setup() - Failed to open /dev/mem\n");
		return NULL;
	}
}

void pwm_setup()
{
	// Set up pwm registers
}

// read_reg IS FIXED
uint32_t read_reg(volatile void **mempp, int offset)
{
	int flag = 1;	// This will go to 0 when function input is invalid

	if (!offsetIsValid(offset))
	{
		// invalid offset
		printf("[ERROR] read_reg() - Register offset of %d is invalid. Allowed register offsets are 0 through %d inclusive and must be aligned.\n", offset, GPIO_REG_MAX_OFFSET);
		flag = 0;
	}

	if (flag == 1)
	{
		volatile uint32_t *reg_ptr = *mempp + offset;
		return *reg_ptr;
	}
	else
		return 0;
}

// edit_reg_bits
// SUMMARY: Edit register by individual bits
// INPUTS:
//     mempp  - address of memory location base pointer
//     offset - offset of the register from the base memory location
//     bits   - the pattern of the bits to replace in the register
//     length - number of bits to replace, read from least significant end of bits
//     index  - desired bit index in the register of the least significant bit of bits 
// OUTPUTS:
//     -1 - write not successful
//      0 - invalid input
//      1 - write successful
int edit_reg_bits(volatile void **mempp, int offset, uint32_t bits, int length, int index)
{
    // Validate inputs
    // mempp
    if (*mempp == NULL)
        return 0;

    // offset
    if (!offsetIsValid(offset))
        return 0;

    // bits (always valid)
    
    // length
    if (length < 0 || length > 32)
		return 0;

    // index
	if (index < 0 || (index + length - 1) > 31)
		return 0;

    // Functional code
    uint32_t mask = 0;
    for (int i = 0; i < length; i++)
        mask |= 0x1 << i;	// Create mask for clearing upper bits
    bits &= mask;	// Clear bits further than length provided

    volatile uint32_t *reg = *mempp + offset;	// Pointer to the register
    uint32_t nextState = *reg;	// Get the current state to transform to next state

    nextState &= ~mask << index; // Clear the bits
    nextState |= bits << index;	// Set the bits
    *reg = nextState;	// Write to register
    if (*reg == nextState)
        return 1;	// Write successful
    else
        return -1;	// Write failed
    
}


void printlnBin32(uint32_t data)
{
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 8; j++)
		{
		printf("%x", data >> 31);	// Grab the MSB
		data <<= 1;					// Shift left for new MSB
		}
		printf(" ");				// Space between bytes
	}
	printf("\n");
}

int pinMode(int pin, int mode)
{
    // Setup gpio_ptr if it is not yet setup
    if (gpio_ptr == NULL)
        gpio_ptr = gpio_setup();

	int flag = 1;

	// Error checking
	if (!pinIsValid(pin))
	{
		printf("[ERROR] pinMode() - Pin number of %d is invalid. Allowed pin numbers are 0 through 27 inclusive.\n", pin);
		flag = 0;
	}
	if (mode < INPUT || mode > ALTFUNC3)	// INPUT is lowest, ALTFUNC3 is highest
	{
		printf("[ERROR] pinMode() - Provided mode is not allowed.\n");
		flag = 0;
	}

	// Functional code
	if (flag == 1)
	{
		// Determine which GPFSEL register to use
		if (pin <= 9)					// GPFSEL0
			if (edit_reg_bits(&gpio_ptr, GPFSEL0, (uint32_t)mode, GPFSEL_OPT_LEN, GPFSEL_OPT_LEN * pin))
				return 1;
			else
				return 0;
		else if (pin <= 19)				// GPFSEL1
			if (edit_reg_bits(&gpio_ptr, GPFSEL1, (uint32_t)mode, GPFSEL_OPT_LEN, GPFSEL_OPT_LEN * (pin - 10)))
				return 1;
			else
				return 0;
		else if (pin <= MAX_PIN_INDEX) 	// GPFSEL2
			if (edit_reg_bits(&gpio_ptr, GPFSEL2, (uint32_t)mode, GPFSEL_OPT_LEN, GPFSEL_OPT_LEN * (pin - 20)))
				return 1;
			else
				return 0;
		else
			return 0;
	}
	else
		return 0;

}