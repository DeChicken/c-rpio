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
#include "c-rpio.h"		// Constants defined here

// Version
#define C_RPIO_VERSION "2.0.0"

// PHILOSOPHY
// Make any functions a user might touch easy to debug

// Allowed PWM pins (I believe these are the same across all Raspberry Pi models?)
int pwm_pins[4] = {12, 13, 18, 19};

// Pointer to the gpio physical memory, null until mem_setup() is run
static volatile void *gpio_ptr = NULL;

// Pointer to the pwm physical memory, null until pwm_setup() is run
static volatile void *pwm_ptr = NULL;

static int offsetIsValid(int offset)
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

static volatile void *mem_setup(off_t base_addr, size_t length)
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
			length,					// Length of mapping
			PROT_READ | PROT_WRITE,	// Enable reading only, for now...
			MAP_SHARED, 			// Share map space with other processes
			memfd,					// File (device) to map from
			base_addr 				// Offset to peripheral base
		);
		close(memfd);	// Close stream with /dev/mem
        return result_ptr;
	}
	else
	{
		printf("[ERROR] mem_setup() - Failed to open /dev/mem\n");
		return NULL;
	}
}

//-------------------------------------- GPIO --------------------------------------//

// read_reg
// SUMMARY: Read a register
// INPUTS:
//     mempp  - address of memory location base pointer
//     offset - offset of the register from the base memory location
// OUTPUTS:
//     returns the 32-bit value in the specified registar as a uint32_t
static uint32_t read_reg(volatile void **mempp, int offset)
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
static int edit_reg_bits(volatile void **mempp, int offset, uint32_t bits, int length, int index)
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

    nextState &= ~(mask << index); // Clear the bits
    nextState |= bits << index;	// Set the bits
    *reg = nextState;	// Write to register
    if (*reg == nextState)
        return 1;	// Write successful
    else
        return -1;	// Write failed
    
}

// printlnBin32
// SUMMARY: Print a 32-bit value in binary (separating bytes)
// INPUTS:
//     data - the 32-bit value to print
// OUTPUTS:
//     void
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

// pinMode
// SUMMARY: Set a GPIO pin mode
// INPUTS:
//     pin  - the GPIO pin (on the header)
//     mode - the mode to set
// OUTPUTS:
//     -1 - failed to set mode
//      0 - invalid inputs
//      1 - mode set successfully
int pinMode(int pin, int mode)
{
    // Setup gpio_ptr if it is not yet setup
    if (gpio_ptr == NULL)
        gpio_ptr = mem_setup(GPIO_BASE, GPIO_REG_MAX_OFFSET + 3);

	int flag = 1;

	// Error checking
	if (!pinIsValid(pin))
	{
		printf("[ERROR] pinMode() - Pin number of %d is invalid. Allowed pin numbers are 0 through 27 inclusive.\n", pin);
		flag = 0;
	}
	if (mode < INPUT || mode > ALT3)	// INPUT is lowest, ALTFUNC3 is highest
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

// digitalRead
// SUMMARY: Read the value of a GPIO pin
// INPUTS:
//     pin - the GPIO pin to read
// OUTPUTS:
//     -1 - input invalid
//      0 - pin is LOW
//      1 - pin is HIGH
int digitalRead(int pin)
{
	// Read the value of pin from GPLEV0
	// NOTE: There is a GPLEV1 register, but only GPIO pins 0-27 are available on the header.

	// Verify inputs
	int flag = 1;	// If 1 at end of checks, run function. If not 1, error
	if (!pinIsValid(pin))
	{
		printf("[ERROR] digitalRead() - Pin number of %d is invalid. Allowed pin numbers are 0 through 27 inclusive.\n", pin);
		flag = 0;
	}
	
	if (flag == 1)
	{
		// Read the register
		return (read_reg(&gpio_ptr, GPLEV0) >> pin) & 0x1;	// Shift right <pin> times, then and with 1 (32-bit)
	}
	else
		return -1;

}

// digitalWrite
// SUMMARY: Write a digital value to a GPIO pin
// INPUTS:
//     pin   - the GPIO pin to write to
//     level - the digital value to write
// OUTPUTS:
//     -1 - failed to write
//      0 - invalid inputs
//      1 - write successful
int digitalWrite(int pin, int level)
{
	// Setup gpio_ptr if it is not yet setup
    if (gpio_ptr == NULL)
        gpio_ptr = mem_setup(GPIO_BASE, GPIO_REG_MAX_OFFSET + 3);

	// Error checking
	int flag = 1;
	if (!pinIsValid(pin))
	{
		printf("[ERROR] digitalWrite() - Pin number of %d is invalid. Allowed pin numbers are 0 through 27 inclusive.\n", pin);
		flag = 0;
	}
	if (level != HIGH && level != LOW)
	{
		printf("[ERROR] digitalWrite() - Level is invalid. Allowed levels are 0 or 1.\n");
		flag = 0;
	}

	// Functional code
	if (flag == 1)
	{
		if (level == HIGH)	// Level is HIGH
		{
			edit_reg_bits(&gpio_ptr, GPSET0, HIGH, 1, pin);
		}
		else if (level == LOW)	// Level is LOW
		{
			edit_reg_bits(&gpio_ptr, GPCLR0, HIGH, 1, pin);
		}
		else
			return -1;	// This should never happen
	}
	else
		return 0;
}

//-------------------------------------- PWM --------------------------------------//

// pwm_cfg
// SUMMARY: Set the PWM0 CTL register
// INPUTS:
//     channel - the pwm channel
//     ~ remaining inputs - see the BCM peripherals datasheet for your Raspberry Pi (PWM section)
// OUTPUTs:
//     -1 - write not successful
//      0 - invalid inputs
//      1 - write successful
static int pwm_cfg(int channel, int PWEN, int MODE, int RPTL, int SBIT, int POLA, int USEF, int MSEN)
{
	// Setup pwm_ptr if it has not already been done
	if (pwm_ptr == NULL)
		pwm_ptr = mem_setup(PWM_BASE, PWM_REG_MAX_OFFSET + 3);

	// Verify inputs 
	// channel
	if ( !(channel == 1 || channel == 2) )	// Channel is neither 1 or 2
		return 0;	// Invalid input
	
	// Rest of inputs(all should either be HIGH or LOW)
	if ( 	!(PWEN == HIGH || PWEN == LOW) ||
			!(MODE == HIGH || MODE == LOW) ||
			!(RPTL == HIGH || RPTL == LOW) ||
			!(SBIT == HIGH || SBIT == LOW) ||
			!(POLA == HIGH || POLA == LOW) ||
			!(USEF == HIGH || USEF == LOW)		)
		return 0;	// Invalid input

	
	// Build config to write
	uint32_t cfg = 0x0;

	cfg |= MSEN;
	cfg <<= 2;
	cfg |= USEF;
	cfg <<= 1;
	cfg |= POLA;
	cfg <<= 1;
	cfg |= SBIT;
	cfg <<= 1;
	cfg |= RPTL;
	cfg <<= 1;
	cfg |= MODE;
	cfg <<= 1;
	cfg |= PWEN;

	// Setup any final inputs to be used in writing to CTL
	int channel_ctl_index = 0;	// Channel 1 bits in CTL start at index 0
	int channel_ctl_len = 8;

	//int edit_reg_bits(volatile void **mempp, int offset, uint32_t bits, int length, int index)
	if (channel == 2)
	{
		channel_ctl_index = 8;	// Channel 2 bits in CTL start at index 8
	}
	
	// Write to CTL register
	return edit_reg_bits(&pwm_ptr, CTL, cfg, channel_ctl_len, channel_ctl_index);	// Write to CTL
}


// pwm_cfg_clr
// SUMMARY: Clear the PWM0 CTL register
// INPUTS:
//     channel - the pwm channel
// OUTPUTs:
//     -1 - write not successful
//      0 - invalid inputs
//      1 - write successful
static int pwm_cfg_clr(int channel)
{
	return pwm_cfg(channel, 0, 0, 0, 0, 0, 0, 0);
}

// analogWrite
// SUMMARY: Write an analog value to a pin
// INPUTS:
//     pin   - the GPIO pin to write to
//     value - the analog value to write
// OUTPUTs:
//     -1 - write not successful
//      0 - invalid inputs
//      1 - write successful
int analogWrite(int pin, int value)
{
	// Setup pwm_ptr if it has not already been done
	if (pwm_ptr == NULL)
		pwm_ptr = mem_setup(PWM_BASE, PWM_REG_MAX_OFFSET + 3);
	
	int flag = 1;

	// Verify inputs
	// pin
	if (!pinIsValid(pin))
	{
		printf("[ERROR] analogWrite() - Pin number of %d is invalid. Allowed pin numbers are 0 through 27 inclusive.\n", pin);
		flag = 0;
	}

	// value
	if (value < 0 || value >= PWM_DEFAULT_RANGE)
	{
		printf("[ERROR] analogWrite() - Value of %d is invalid. Allowed values are 0 through %d inclusive.\n", value, PWM_DEFAULT_RANGE - 1);
		flag = 0;
	}

	// Functional code
	if (flag == 1)
	{
		// Set up write parameters that vary per channel
		int channel = 0;
		int rng_reg = -1;
		int dat_reg = -1;
		if (pin == 12 || pin == 18)
		{
			channel = 1;
			rng_reg = RNG1;
			dat_reg = DAT1;
		}
		else if (pin == 13 || pin == 19)
		{
			channel = 2;
			rng_reg = RNG2;
			dat_reg = DAT2;
		}

		// Set the range to default
		edit_reg_bits(&pwm_ptr, rng_reg, PWM_DEFAULT_RANGE, REG_LEN_BITS, 0);

		// Set the DAT register for the inputted channel
		edit_reg_bits(&pwm_ptr, dat_reg, value, REG_LEN_BITS, 0);

		int aw_cfg[7] = {
			1, // PWEN
			0, // MODE
			0, // RPTL
			0, // SBIT
			0, // POLA
			0, // USEF
			1, // MSEN
		};

		// Set up CTL for M/S mode and begin PWM
		return pwm_cfg(channel, aw_cfg[0], aw_cfg[1], aw_cfg[2], aw_cfg[3], aw_cfg[4], aw_cfg[5], aw_cfg[6]);
	}
	else
		return 0;	// Invalid inputs
}

// pwm_dump
// SUMMARY: Dump various PWM0 register contents
// INPUTS:
//     void
// OUTPUTS:
//     void
static void pwm_dump(void)
{
	printf("\npwm_dump:\n");
	printf("CTL register:   "); printlnBin32(read_reg(&pwm_ptr, CTL));
	printf("STA register:   "); printlnBin32(read_reg(&pwm_ptr, STA));
	printf("RNG1 register:  "); printlnBin32(read_reg(&pwm_ptr, RNG1));
	printf("DAT1 register:  "); printlnBin32(read_reg(&pwm_ptr, DAT1));
	printf("RNG2 register:  "); printlnBin32(read_reg(&pwm_ptr, RNG2));
	printf("DAT2 register:  "); printlnBin32(read_reg(&pwm_ptr, DAT2));
	printf("FIF1 register:  "); printlnBin32(read_reg(&pwm_ptr, FIF1));
}
