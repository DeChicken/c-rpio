#include <stdio.h>      // Standard input/output definitions
#include <unistd.h>     // UNIX standard function defs
#include <string.h>
#include <fcntl.h>      // File control defs
#include <errno.h>      // Error number defs
#include <termios.h>   // POSIX terminal control defs
#include <sys/ioctl.h>  // Device control system call
#include <poll.h>

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

    // Poll the fd
    struct pollfd src;      // Struct used by poll()
    src.fd = fd;
    src.events = POLLIN;
    src.revents = 0;

    int check = poll(&src, 1, -1);
    printf("check is %d\n", check);

    // Non-blocking read
    fcntl(fd, F_SETFL, FNDELAY);
    int c;
    if (read(fd, &c, 1) == -1)
        printf("read() returned in error:\n\terrno is %d\n", errno);
    else
        printf("c is \'%c\' as a char, 0x%x in hex\n", c, c);

    close(fd);
}
