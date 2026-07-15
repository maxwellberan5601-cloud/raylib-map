
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "CoreSLAM.h"

#define LD06_SCAN_SIZE 503
#define LINE_SIZE 100



//gcc Slam_Controller.c CoreSLAM_state.c CoreSLAM_random.c CoreSLAM.c CoreSLAM_ext.c CoreSLAM_loop_closing.c -lm -o go
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



ts_sensor_data_t sd_a = {
    .timestamp = 0,
    //current readings
    .q1 = 0,
    .q2 = 0,
    .v = 0,
    .psidot = 0,
    .position = {0},
    .d = {0},
};

ts_sensor_data_t sd_b = {
    .timestamp = 0,
    //current readings
    .q1 = 0,
    .q2 = 0,
    .v = 0,
    .psidot = 0,
    .position = {0},
    .d = {0},
};

ts_sensor_data_t *write_scan = &sd_a;


ts_scan_t scan = {
    .x = {0},
    .y = {0},
    .value = {0},
    .nb_points = 0
};



ts_state_t state = {
    .randomizer = {0},
    .map = NULL,
    .params = {0},
    .laser_params = {
        .scan_size = 503,
        .angle_min = 0,
        .angle_max = 360,
        .detection_margin = 0,
        .distance_no_detection = 0
    },
    .position = {0},
    //pervious readings
    .q1 = 0,
    .q2 = 0,
    .timestamp = 0,
    .psidot = 0.0,
    .v = 0.0,
    .distance = 0.0,
    .hole_width = 0,
    .direction = 0,
    .done = 0,
    .draw_hole_map = 0,
    .scan = {0},
    .sigma_xy = 0.0,
    .sigma_theta = 0.0
};

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
    int ready_process = 0;

    printf("Reading serial data from %s...\n", port_name);

    while (1) {

        line_received = serial_read_line(serial_port, line, LINE_SIZE);

        if (line_received < 0) {
            break;
        }

        if (line_received == 1) {
            if (extract_first_number(line, &i, &angle, &distance)) {
                // printf("(%d, %d)\n", i, distance);
            } else {
                printf("Invalid line: %s\n", line);
            }
        }
        write_scan->d[i] = distance;

        if (i == 502) {
            ts_build_scan(write_scan, &scan, &state, 1);
            for(i = 0; i < 503; i++) {
                printf("(i = %d x:%lf y:%lf, value: %d)\n", i ,scan.x[i], scan.y[i], scan.value[i]);
            }
            // ts_sensor_data_t *swaping_buffer = read_scan
            // write_scan = read_scan;
            // read_scan = write_scan;
        }
    }

    close(serial_port);
    return 0;
}
