#include <stdio.h>      // Standard input/output definitions
#include <unistd.h>     // UNIX standard function defs
#include <fcntl.h>      // File control defs
#include <errno.h>      // Error number defs
#include <termios.h>   // POSIX terminal control defs
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
    /*
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
    */
    struct termios serialSet;
    memset(&serialSet, 0, sizeof(serialSet));
    serialSet.c_iflag = IGNBRK;
    serialSet.c_cflag = CS8 | CREAD | CLOCAL;
    memset(serialSet.c_cc, _POSIX_VDISABLE, NCCS);
    serialSet.c_cc[VMIN] = 0;
    serialSet.c_cc[VTIME] = 0;
    cfsetispeed(&serialSet, baud_rate);        // Set input expected speed to baud_rate
    cfsetospeed(&serialSet, baud_rate);        // Set output expected speed to baud_rate
    if ()
        printf("Failed to set up serial port\n");
    return 0;
}


int main(void)
{
    // Open serial port
    int fd = open_port();
    printf("fd is %x\n", fd);

    // Set up serial port
    set_up_port(fd, B1800);

    printf("Reading from /dev/ttyAMA1...\n");
    char c;
    if (read(fd, &c, 1) < 0)
        printf("Error reading from /dev/ttyAMA1\n");
    else
        printf("c is \"%s\"\n", c);
    close(fd);
}
