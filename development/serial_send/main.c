#include <stdio.h>      // Standard input/output definitions
#include <unistd.h>     // UNIX standard function defs
#include <string.h>
#include <fcntl.h>      // File control defs
#include <errno.h>      // Error number defs
#include <termios.h>    // POSIX terminal control defs
#include <sys/ioctl.h>  // Device control system call

int open_port(void)
{
    int fd;     // Initialize file descriptor
    int delay = 250 * 1000; // Delay between program continuation and port opening

    fd = open(
        "/dev/ttyAMA1", 
        O_RDWR | 
        O_NOCTTY | 
        O_NDELAY
        );

    // Test to see if file was opened correctly
    if (fd == -1)
        // Port was not opened...
        printf("open_port: Unable to open /dev/ttyAMA1 - ");
    else
        // Port was opened correctly...
        fcntl(fd, F_SETFL, 0);
    
    usleep(delay);  // Port opening delay (safety)

    return fd;  // Return the file descriptor
}

int set_up_port(int fd, speed_t baud_rate)
{
    struct termios serialSet;
    memset(&serialSet, 0, sizeof(serialSet));
    serialSet.c_iflag = IGNBRK;
    serialSet.c_cflag = CS8 | CREAD | CLOCAL;
    memset(serialSet.c_cc, _POSIX_VDISABLE, NCCS);
    serialSet.c_cc[VMIN] = 0;
    serialSet.c_cc[VTIME] = 0;
    cfsetispeed(&serialSet, baud_rate);        // Set input expected speed to baud_rate
    cfsetospeed(&serialSet, baud_rate);        // Set output expected speed to baud_rate
    if (tcsetattr(fd, TCSANOW, &serialSet) == -1)
        printf("Failed to set up serial port\n");
    return 0;
}

int main(void)
{
    // Open serial port
    int fd = open_port();
    printf("fd is %x\n", fd);

    // Set up serial port
    set_up_port(fd, B9600);

    // String to write
    char d = 'o';

    // Write string
    tcflush(fd, TCIOFLUSH);     // Clear input and output buffers on UART
    printf("Writing to /dev/ttyAMA1...\n");
    int n = write(fd, &d, 1);
    if (n < 0)
        printf("write() failed!\n");
    // Close the file
    close(fd);
}

// Used this as a guide: https://www.cmrr.umn.edu/~strupp/serial.html#5_1
// and this: https://raspberrypi.stackexchange.com/questions/57906/uart-interrupt-in-raspberry-pi-with-c-or-c