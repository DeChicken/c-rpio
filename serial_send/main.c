#include <stdio.h>      // Standard input/output definitions
#include <unistd.h>     // UNIX standard function defs
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

int set_up_port(int fd, int baudrate)
{
    // Set baud rate
    struct termios options;                 // Struct to hold termios options
    tcgetattr(fd, &options);                // Fill options with current options
    cfsetispeed(&options, baudrate);        // Set input expected speed to baudrate
    cfsetospeed(&options, baudrate);        // Set output expected speed to baudrate
    options.c_cflag |= (CLOCAL | CREAD);    // Enable the receiver and set local mode...
    printf("c_lflag = %x\n", options.c_lflag);
    tcsetattr(fd, TCSANOW, &options);       // Set the new options for the port...

    // Buffer for status bits
    int status;
    ioctl(fd, TIOCMGET, &status);       // Get the status bits
    printf("status = %x\n", status);    // Print the status bits
    return 0;
}

int main(void)
{
    // Open serial port
    int fd = open_port();
    printf("fd is %x\n", fd);

    // Set up serial port
    set_up_port(fd, B1800);

    // String to write
    char *d = "o\n";

    // Write string
    tcflush(fd, TCIOFLUSH);     // Clear input and output buffers on UART
    printf("Writing to /dev/ttyAMA1...\n");
    int n = write(fd, &d, 2);
    if (n < 0)
        printf("write() of 2 bytes failed!\n");
    // Close the file
    close(fd);
}

// Used this as a guide: https://www.cmrr.umn.edu/~strupp/serial.html#5_1
// and this: https://raspberrypi.stackexchange.com/questions/57906/uart-interrupt-in-raspberry-pi-with-c-or-c