/*
	Purpose:
		WIP: This program drives a gpio pin on the raspberry pi
	Author: David Craig
	Version: 1.0.0
*/

#include <stdio.h>
#include <inttypes.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>

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

// Pointer to the gpio physical memory, null until gpio_setup() is run
static volatile void *gpio_ptr = NULL;

void gpio_setup(void)
{	
	int memfd;
	
	// Attempt to open /dev/mem
	if ((memfd = open("/dev/mem", O_RDWR | O_SYNC)) > 0) // Read only for now...
	/*
		O_SYNC description from the open(2) man page:
			"O_SYNC would also always flush the last
				modification timestamp metadata"
	*/
	{
		
		// Create mapping to the gpio registers
		gpio_ptr = mmap(
			NULL,					// Addr in virtual mem to map to
			GPIO_REG_MAX_OFFSET,	// Length of mapping
			PROT_READ | PROT_WRITE,	// Enable reading only, for now...
			MAP_SHARED, 			// Share map space with other processes
			memfd,					// File (device) to map from
			GPIO_BASE 				// Offset to peripheral base
		);
		close(memfd);	// Close stream with /dev/mem
	}
	else
	{
		printf("[ERROR] gpio_setup() - Failed to open /dev/mem\n");
		exit(EXIT_FAILURE);	// End the program in error
	}
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

static int set_reg(int offset, uint32_t bits, int length, int chgOffset)
{
	// Run gpio_setup if needed
	if (gpio_ptr == NULL)
		gpio_setup();

	// -------------- Error checking ------------------ //
	/* errcode = 	[offsetNA]		// bit 0
					[offsetOOR]		// bit 1
					[bitOOR]		// bit 2
					[levelInv]		// bit 3
					[lengthOOR]		// bit 4
	*/
	int errcode = 0x0;
	if (offset % 4 != 0)	// Offset Not Aligned [offsetNA]
		errcode |= 0x1; // 0b1
	
	if ((offset < 0) || (offset > GPIO_REG_MAX_OFFSET))	// Offset Out of Range [offsetOOR]
		errcode |= 0x2; // 0b10
	
	if (length < 0 || length > 32)	// Length Out of Range [lengthOOR]
		errcode |= 0x4; // 0b100

	if (chgOffset < 0 || (chgOffset + length - 1) > 31)
		errcode |= 0x10; // 0b10000

	// Print error messages
	if (errcode != 0)
	{
		printf("[ERROR] set_reg() - Register set errors occured\n");
		printf("\tError Code: 0x%x\n", errcode);
		for (int i = 0; i < 5; i++)
		{
			if (errcode >> 4)
			switch (i)
			{
				case 4:
					printf("\t- Offset of 0x%x is not aligned. See Sec 5.2 in the BCM2711 Datasheet.\n", offset);
					break;
				case 3:
					printf("\t- Offset of 0x%x is out of range. See Sec 5.2 in the BCM2711 Datasheet.\n", offset);
					break;
				case 2:
					printf("\t- Length of %d is out of range. Allowed lengths are 0 through 32 inclusive.\n", length);
					break;
				case 0:
					printf("\t- Change offset of %d is invalid with length of %d. Length + change offset - 1 must be in register bounds.\n",
						chgOffset,
						length);
					break;
				default:
					printf("\t - [ERROR] Default case reached.\n");
			}
			errcode = (errcode << 1) & 0x1F;	// Shift and crop higher bits
		}
		printf("\n");
		exit(EXIT_FAILURE);
	}
	
	// ---------- Functional Code --------------- //
	
	// Enforce the length provided
	uint32_t mask = 0;
	for (int i = 0; i < length; i++)
		mask |= 0x1 << i;	// Create mask for clearing upper bits
	bits &= mask;	// Clear bits further than length provided

	volatile uint32_t *reg = gpio_ptr + offset;	// Pointer to the register
	uint32_t nextState = *reg;	// Get the current state to transform to next state

	nextState &= ~mask << chgOffset; // Clear the bits
	nextState |= bits << chgOffset;	// Set the bits
	*reg = nextState;	// Write to register
	if (*reg == nextState)
		return 1;	// Write successful
	else
		return 0;	// Write failed
	
}

void printGPIO(void)
{
	// Run gpio_setup if needed
	if (gpio_ptr == NULL)
		gpio_setup();

	volatile uint32_t *p;
	for (int i = 0; i < GPIO_REG_MAX_OFFSET; i++)
	{
		p = (volatile uint32_t *)(gpio_ptr + i);
		switch (i)
		{
			case 0x00:
				printf("GPFSEL0 = 0b ");
				printlnBin32(*p);
				break;
			case 0x04:
				printf("GPFSEL1 = 0b ");
				printlnBin32(*p);
				break;
			case 0x08:
				printf("GPFSEL2 = 0b ");
				printlnBin32(*p);
				break;
			case 0x0c:
				printf("GPFSEL3 = 0b ");
				printlnBin32(*p);
				break;
			case 0x10:
				printf("GPFSEL4 = 0b ");
				printlnBin32(*p);
				break;
			case 0x14:
				printf("GPFSEL5 = 0b ");
				printlnBin32(*p);
				break;
			case 0x1c:
				printf("GPSET0  = 0b ");
				printlnBin32(*p);
				break;
			case 0x20:
				printf("GPSET1  = 0b ");
				printlnBin32(*p);
				break;
			case 0x28:
				printf("GPCLR0  = 0b ");
				printlnBin32(*p);
				break;
			case 0x2C:
				printf("GPCLR1  = 0b ");
				printlnBin32(*p);
				break;
		}
	}
}

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

int pinMode(int pin, int mode)
{
	/* Stages:
		- Error checking
			- pin must be allowed
			- check if mode is allowed
		- Register setting
			- use set_reg() and predefined constants ONLY
	*/

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
			if (set_reg(GPFSEL0, (uint32_t)mode, GPFSEL_OPT_LEN, GPFSEL_OPT_LEN * pin))
				return 1;
			else
				return 0;
		else if (pin <= 19)				// GPFSEL1
			if (set_reg(GPFSEL1, (uint32_t)mode, GPFSEL_OPT_LEN, GPFSEL_OPT_LEN * (pin - 10)))
				return 1;
			else
				return 0;
		else if (pin <= MAX_PIN_INDEX) 	// GPFSEL2
			if (set_reg(GPFSEL2, (uint32_t)mode, GPFSEL_OPT_LEN, GPFSEL_OPT_LEN * (pin - 20)))
				return 1;
			else
				return 0;
		else
			return 0;
	}
	else
		return 0;

}

int digitalWrite(int pin, int level)
{
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
			if(set_reg(GPSET0, HIGH, 1, pin))
				return 1;
			else
				return 0;
		}
		else if (level == LOW)	// Level is LOW
		{
			if(set_reg(GPCLR0, HIGH, 1, pin))
				return 1;
			else
				return 0;
		}
		else
			return 0;
	}
	else
		return 0;
}

uint32_t read_reg(int offset)
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
		volatile uint32_t *reg_ptr = gpio_ptr + offset;
		return *reg_ptr;
	}
	else
		return 0;
}

int digitalRead(int pin)
{
	// Read the value of pin from GPLEV0
	// NOTE: There is a GPLEV1 register, but only GPIO pins 0-27 are available to the user.

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
		return (read_reg(GPLEV0) >> pin) & 0x1;	// Shift right <pin> times, then and with 1 (32-bit)
	}
	else
		return -1;

}
