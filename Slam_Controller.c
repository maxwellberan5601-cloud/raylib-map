
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>

#include "CoreSLAM.h"
#include "output_manager.h"

#define LD06_SCAN_SIZE 503
#define LINE_SIZE 100
#define LD06_NO_DETECTION_DISTANCE_MM 6000
#define MIN_SCAN_POINTS 120



//gcc Slam_Controller.c CoreSLAM_state.c CoreSLAM_random.c CoreSLAM.c CoreSLAM_ext.c CoreSLAM_loop_closing.c -lm -o go
//./go

//powershell

//usbipd list
//usbipd bind --busid <BUSID>
//usbipd attach --wsl --busid <BUSID>
// ls /dev/ttyACM* /dev/ttyUSB*

// gcc raylib_map_viewer.c Slam_Controller.c output_manager.c CoreSLAM_state.c CoreSLAM_random.c CoreSLAM.c CoreSLAM_ext.c CoreSLAM_loop_closing.c -o slam_viewer -lraylib -lGL -lm -lpthread -ldl -lrt -lX11
// ./slam_viewer

/*
 * Opens and configures the serial port.
 * Returns the serial file descriptor, or -1 on error.
 */
int serial_init(const char *port_name)
{
    int serial_port;
    struct termios tty;

    serial_port = open(port_name, O_RDONLY | O_NOCTTY | O_NONBLOCK);
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
    tty.c_cc[VMIN] = 0;
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
 * Returns 1 for a complete line, 2 for a byte read, 0 for no data, or -1 on error.
 */
int serial_read_line(int serial_port, char line[], int line_size)
{
    static int position = 0;
    char character;
    int bytes_read = read(serial_port, &character, 1);

    if (bytes_read < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0;
        }
        perror("Serial read failed");
        return -1;
    }

    if (bytes_read == 0) {
        return 0;
    }

    if (character == '\r') {
        return 2;
    }

    if (character == '\n') {
        line[position] = '\0';
        position = 0;
        return 1;
    }

    if (position < line_size - 1) {
        line[position] = character;
        position++;
        return 2;
    } else {
        position = 0;
        printf("Serial line was too long\n");
        return 2;
    }
}

/*
 * Extracts the first number from a line such as hello:3,4,5.
 */
int extract_first_number(const char line[], int *i, float *angle, 
                            int *distance, int *q1, int *q2, unsigned long *timestamp)
{   
    int quality;
    int point_number;

    if (sscanf(line, "time: %lu", timestamp) == 1) {
        return 1;
    }
    else if (sscanf(line, "Index:%d:%d", q1, q2) == 2) {
        return 2;
    }
    else if (sscanf(line, "%d,%f,%d", &point_number, angle, distance) == 3) {
        while (*angle < 0.0f) {
            *angle += 360.0f;
        }
        while (*angle >= 360.0f) {
            *angle -= 360.0f;
        }

        *i = (int)((*angle / 360.0f) * LD06_SCAN_SIZE);
        if (*i >= LD06_SCAN_SIZE) {
            *i = LD06_SCAN_SIZE - 1;
        }
        return 3;
    }
    else if (sscanf(line, "%f,%d,%d,%d,%d", angle, distance, &quality, q1, q2) == 5) {
        while (*angle < 0.0f) {
            *angle += 360.0f;
        }
        while (*angle >= 360.0f) {
            *angle -= 360.0f;
        }

        *i = (int)((*angle / 360.0f) * LD06_SCAN_SIZE);
        if (*i >= LD06_SCAN_SIZE) {
            *i = LD06_SCAN_SIZE - 1;
        }
        return 3;
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


ts_state_t state = {0};

ts_sensor_data_t *write_scan = &sd_a;


ts_scan_t scan = {
    .x = {0},
    .y = {0},
    .value = {0},
    .nb_points = 0
};

ts_map_t map = {
    .map = {0}
};

ts_robot_parameters_t params = {
    .r = 0.061/2,
    .R = 0.280/2,
    .inc = 663*2, //wheel increments per turn
    .ratio = 1, //ratio bewteen right and left wheel (if there not quiet the same size)
};

ts_position_t position = {
    .x = 10000,
    .y = 10000,
    .theta = 359, //I'm pretty sure it starts there
};

ts_laser_parameters_t laser_params = {
        .offset = 0, //it assumes the lidars offset is proptional in x y from yhe centre
        .scan_size = 503,
        .angle_min = 0,
        .angle_max = 360,
        .detection_margin = 0, //because we don't want to ignore any values at beging of 0 or end of 360
        .distance_no_detection = LD06_NO_DETECTION_DISTANCE_MM //default value when laser returns 0 because its too far
};

double sigma_xy = 1000; //100mm range +- uncertainty
double sigma_theta = 30; //+- degree uncertainity
int hole_width = 300; //thickness of obstacles 300 mm should be thick to not look fragmented
    //but small enough not artically make the obstalces bigger
int direction = TS_DIRECTION_FORWARD;
//for live mapping


//values will be init to unkown by map_init




static int controller_serial_port = -1;
static char controller_line[LINE_SIZE];
static int scan_ready = 0;
static int scan_points_received = 0;
static unsigned char scan_received[LD06_SCAN_SIZE] = {0};

static void clear_pending_scan(void)
{
    for (int i = 0; i < LD06_SCAN_SIZE; i++) {
        write_scan->d[i] = 0;
        scan_received[i] = 0;
    }
    scan_points_received = 0;
}

static void process_controller_line(const char *line)
{
    int i = -1;
    float angle = 0;
    int distance = 0;
    int q1 = 0;
    int q2 = 0;
    unsigned long timestamp = 0;
    int line_type = extract_first_number(line, &i, &angle, &distance, &q1, &q2, &timestamp);

    if (line[0] == '\0') {
        if (scan_points_received >= MIN_SCAN_POINTS) {
            scan_ready = 1;
        }
    } else if (line_type == 1) {
        write_scan->timestamp = timestamp;
    } else if (line_type == 2) {
        write_scan->q1 = q1;
        write_scan->q2 = q2;
    } else if (line_type == 3) {
        if (i >= 0 && i < LD06_SCAN_SIZE) {
            write_scan->d[i] = distance;

            if (!scan_received[i]) {
                scan_received[i] = 1;
                scan_points_received++;
            }
        }
    } else {
        printf("Invalid line: %s\n", line);
    }
}

static void run_slam_when_ready(void)
{
    if (!scan_ready) {
        return;
    }

    if (write_scan->timestamp == state.timestamp) {
        write_scan->timestamp = state.timestamp + 100000;
    }

    if (write_scan->timestamp == 0) {
        return;
    }

    ts_iterative_map_building(write_scan, &state);
    set_map(state.map);
    set_position(state.position);
    scan_ready = 0;
    clear_pending_scan();
}

int slam_controller_init(const char *port_name)
{
    controller_serial_port = serial_init(port_name);
    if (controller_serial_port < 0) {
        return 0;
    }

    printf("Reading serial data from %s...\n", port_name);

    ts_map_init(&map);
    ts_state_init(&state, &map, &params,
                  &laser_params, &position, sigma_xy,
                  sigma_theta, hole_width, direction);
    clear_pending_scan();
    set_map(state.map);
    set_position(state.position);
    return 1;
}

void slam_controller_update(void)
{
    int lines_processed = 0;

    if (controller_serial_port < 0) {
        return;
    }

    while (lines_processed < 4096) {
        int line_received = serial_read_line(controller_serial_port, controller_line, LINE_SIZE);

        if (line_received < 0) {
            close(controller_serial_port);
            controller_serial_port = -1;
            return;
        }

        if (line_received == 0) {
            break;
        }

        if (line_received == 1) {
            process_controller_line(controller_line);
            run_slam_when_ready();
        }

        lines_processed++;
    }
}

void slam_controller_close(void)
{
    if (controller_serial_port >= 0) {
        close(controller_serial_port);
        controller_serial_port = -1;
    }
}
