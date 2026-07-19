
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "CoreSLAM.h"
#include "output_manager.h"

#define LD06_SCAN_SIZE 503
#define LINE_SIZE 100
#define LD06_NO_DETECTION_DISTANCE_MM 6000



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
int extract_first_number(const char line[], int *i, float *angle, 
                            int *distance, int *q1, int *q2, unsigned long *timestamp)
{   
    int quality;

    if (sscanf(line, "time: %lu", timestamp) == 1) {
        return 1;
    }
    else if (sscanf(line, "Index:%d:%d", q1, q2) == 2) {
        return 2;
    }
    else if (sscanf(line, "%d,%f,%d", i,angle,distance) == 3) {
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
    .inc = 663, //wheel increments per turn
    .ratio = 1, //ratio bewteen right and left wheel (if there not quiet the same size)
};

ts_position_t position = {
    .x = 0,
    .y = 0,
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

double sigma_xy = 100; //100mm range +- uncertainty
double sigma_theta = 10; //+- degree uncertainity
int hole_width = 300; //thickness of obstacles 300 mm should be thick to not look fragmented
    //but small enough not artically make the obstalces bigger
int direction = TS_DIRECTION_FORWARD;
//for live mapping


//values will be init to unkown by map_init




int main(void)
{
    const char *port_name = "/dev/ttyACM1";
    int serial_port;
    char line[LINE_SIZE];
    int i = -1;
    float angle = 0;
    int distance = 0;

    int q1 = 0;
    int q2 = 0;

    unsigned long timestamp = 0;

    int line_received;
    int scan_ready = 0;
    unsigned long fallback_timestamp = 0;


    serial_port = serial_init(port_name);
    if (serial_port < 0) {
        return 1;
    }
    int ready_process = 0;

    printf("Reading serial data from %s...\n", port_name);

    //slam
    //set the map to be filled with uknowns
    ts_map_init(&map);
    ts_state_init(&state, &map, &params, 
                &laser_params, &position, sigma_xy, 
                sigma_theta, hole_width, direction);
    



    while (1) {

        line_received = serial_read_line(serial_port, line, LINE_SIZE);

        if (line_received < 0) {
            break;
        }

        if (line_received == 1) {
            int line_type = extract_first_number(line, &i, &angle, &distance, &q1, &q2, &timestamp);

            if (line_type == 1) {
                write_scan->timestamp = timestamp;
            } else if (line_type == 2) {
                write_scan->q1 = q1;
                write_scan->q2 = q2;
            } else if (line_type == 3) {
                // printf("(%d, %d)\n", i, distance);
                if (i >= 0 && i < LD06_SCAN_SIZE) {
                    write_scan->d[i] = distance;

                    if (i == LD06_SCAN_SIZE - 1) {
                        scan_ready = 1;
                    }
                }
            } else {
                printf("Invalid line: %s\n", line);
            }
        }


        // currently writing over previous scans
        // if (i == 502) {
        //     ts_build_scan(write_scan, &scan, &state, 1);
        //     for(i = 0; i < 503; i++) {
        //         printf("(i = %d x:%lf y:%lf, value: %d)\n", i ,scan.x[i], scan.y[i], scan.value[i]);
        //     }
        // }


        //slam

        //only care about feeding sd and ts_scan only important if I want to make save past scans
        //for loop closure in sensordata[i]

        //raylib
        //I want to output postion state state->postion.x state->postion.y state->postion.theta
        //and map

        //missing q1 q2 update
        //and time stamp for sensor data
        if (scan_ready && write_scan->timestamp == state.timestamp) {
            fallback_timestamp = state.timestamp + 100000;
            write_scan->timestamp = fallback_timestamp;
        }

        if (scan_ready && write_scan->timestamp != 0) {
            ts_iterative_map_building(write_scan, &state);

            // state.map is already a pointer to the map.
            set_map(state.map);
            set_position(state.position);
            scan_ready = 0;
            double x = state.position.x;
            double y = state.position.y;
            printf("x: %lf, y: %lf\n", x, y);

        }
        //why doe it need to be static







    }

    close(serial_port);
    return 0;
}
