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

// PHILOSOPHY
// Make any functions a user might touch easy to debug

// Allowed PWM pins (I believe these are the same across all Raspberry Pi models?)
int pwm_pins[4] = {12, 13, 18, 19};

// Pointer to the gpio physical memory, null until mem_setup() is run
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

volatile void *mem_setup(off_t base_addr, size_t length)
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

    nextState &= ~(mask << index); // Clear the bits
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

int pwm_cfg(int channel, int PWEN, int MODE, int RPTL, int SBIT, int POLA, int USEF, int MSEN)
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

	printf("cfg is : "); printlnBin32(cfg);
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

int pwm_cfg_clr(int channel)
{
	//int pwm_cfg(int channel, int PWEN, int MODE, int RPTL, int SBIT, int POLA, int USEF, int MSEN)
	pwm_cfg(channel, 0, 0, 0, 0, 0, 0, 0);
}

// pin: the Arduino pin to write to. Allowed data types: int.
// value: the duty cycle: between 0 (always off) and 255 (always on). Allowed data types: int.
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
		printf("RNG register for channel %d before write: 0x%x\n", channel, read_reg(&pwm_ptr, rng_reg));
		edit_reg_bits(&pwm_ptr, rng_reg, PWM_DEFAULT_RANGE, REG_LEN_BITS, 0);
		printf("RNG register for channel %d after write:  0x%x\n", channel, read_reg(&pwm_ptr, rng_reg));

		// Set the DAT register for the inputted channel
		printf("DAT register for channel %d before write: 0x%x\n", channel, read_reg(&pwm_ptr, dat_reg));
		edit_reg_bits(&pwm_ptr, dat_reg, value, REG_LEN_BITS, 0);
		printf("DAT register for channel %d after write:  0x%x\n", channel, read_reg(&pwm_ptr, dat_reg));

		// I have access to the PWM0 module 

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
		printf("CTL register before write: "); printlnBin32(read_reg(&pwm_ptr, CTL));
		pwm_cfg(channel, aw_cfg[0], aw_cfg[1], aw_cfg[2], aw_cfg[3], aw_cfg[4], aw_cfg[5], aw_cfg[6]);
		printf("CTL register after write:  "); printlnBin32(read_reg(&pwm_ptr, CTL));
		printf("GPFSEL1 register:  "); printlnBin32(read_reg(&gpio_ptr, GPFSEL1));

		return 1;
	}
	else
		return 0;	// Invalid inputs
}

void pwm_dump(void)
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

void mini_main(void)
{
	pwm_dump();
	edit_reg_bits(&pwm_ptr, CTL, 0, 32, 0);
}
