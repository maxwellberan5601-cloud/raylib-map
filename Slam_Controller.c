
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#define LD06_SCAN_SIZE 503
#define LINE_SIZE 100

uint16_t d[LD06_SCAN_SIZE];

//gcc Slam_Controller.c -o go
//./go

//powershell
//usbipd list
//usbipd bind --busid <BUSID>
//usbipd attach --wsl --busid <BUSID>
// ls /dev/ttyACM* /dev/ttyUSB*


/*
 * Opens and configures the serial port.
 * Returns the serial file descriptor, or -1 on error.
 */
int serial_init(const char *port_name)
{
    int serial_port;
    struct termios tty;

    serial_port = open(port_name, O_RDONLY | O_NOCTTY);
    if (serial_port < 0) {
        perror("Could not open serial port");
        return -1;
    }

    if (tcgetattr(serial_port, &tty) != 0) {
        perror("Could not read serial settings");
        close(serial_port);
        return -1;
    }

    cfsetispeed(&tty, B230400);
    cfsetospeed(&tty, B230400);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        perror("Could not configure serial port");
        close(serial_port);
        return -1;
    }

    return serial_port;
}

/*
 * Reads characters until a complete line arrives.
 * Returns 1 for a complete line, 0 otherwise, or -1 on error.
 */
int serial_read_line(int serial_port, char line[], int line_size)
{
    static int position = 0;
    char character;
    int bytes_read = read(serial_port, &character, 1);

    if (bytes_read < 0) {
        perror("Serial read failed");
        return -1;
    }

    if (bytes_read == 0) {
        return 0;
    }

    if (character == '\n' || character == '\r') {
        if (position == 0) {
            return 0;
        }

        line[position] = '\0';
        position = 0;
        return 1;
    }

    if (position < line_size - 1) {
        line[position] = character;
        position++;
    } else {
        position = 0;
        printf("Serial line was too long\n");
    }

    return 0;
}

/*
 * Extracts the first number from a line such as hello:3,4,5.
 */
int extract_first_number(const char line[], int *i, float *angle, int *distance)
{
    int result = sscanf(line, "%d,%f,%d", i,angle,distance); 
    //set the values of the pointers

    if (result == 3) {
        return 1;
    }

    return 0;
}

int main(void)
{
    const char *port_name = "/dev/ttyACM0";
    int serial_port;
    char line[LINE_SIZE];
    int i;
    float angle;
    int distance;
    int line_received;

    serial_port = serial_init(port_name);
    if (serial_port < 0) {
        return 1;
    }

    printf("Reading serial data from %s...\n", port_name);

    while (1) {
        line_received = serial_read_line(serial_port, line, LINE_SIZE);

        if (line_received < 0) {
            break;
        }

        if (line_received == 1) {
            if (extract_first_number(line, &i, &angle, &distance)) {
                printf("(%d, %d)\n", i, distance);
            } else {
                printf("Invalid line: %s\n", line);
            }
        }
    }

    close(serial_port);
    return 0;
}
