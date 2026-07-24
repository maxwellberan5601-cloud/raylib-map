#include "raylib.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "CoreSLAM.h"

#define WINDOW_W 1000
#define WINDOW_H 900
#define DEFAULT_SERIAL_PORT "/dev/ttyACM1"
#define SERIAL_BAUD B115200
#define LD06_SCAN_SIZE 503
#define LINE_SIZE 128
#define MIN_SCAN_POINTS 120
#define LD06_NO_DETECTION_DISTANCE_MM 6000
#define MAX_PERSISTENT_POINTS 200000

static ts_sensor_data_t sensor_data = {
    .timestamp = 0,
    .q1 = 0,
    .q2 = 0,
    .v = 0,
    .psidot = 0,
    .position = {0},
    .d = {0},
};

static ts_map_t dummy_map = {0};

static ts_robot_parameters_t params = {
    .r = 0.061 / 2,
    .R = 0.280 / 2,
    .inc = 663 * 2,
    .ratio = 1,
};

static ts_position_t start_position = {
    .x = 0,
    .y = 0,
    .theta = 0,
};

static ts_laser_parameters_t laser_params = {
    .offset = 0,
    .scan_size = LD06_SCAN_SIZE,
    .angle_min = 0,
    .angle_max = 360,
    .detection_margin = 0,
    .distance_no_detection = LD06_NO_DETECTION_DISTANCE_MM,
};

static ts_state_t state = {0};
static ts_scan_t latest_scan = {0};
static unsigned char scan_received[LD06_SCAN_SIZE] = {0};
static int scan_points_received = 0;
static int scan_count = 0;
static int lines_received = 0;
static int lidar_points_parsed = 0;
static double persistent_x[MAX_PERSISTENT_POINTS];
static double persistent_y[MAX_PERSISTENT_POINTS];
static int persistent_points = 0;

static void remember_point_from_bucket(int bucket, int distance)
{
    if (distance <= 0 || distance <= state.hole_width / 2 || persistent_points >= MAX_PERSISTENT_POINTS) {
        return;
    }

    double angle_deg = laser_params.angle_min +
        ((double)bucket) * (laser_params.angle_max - laser_params.angle_min) /
        (laser_params.scan_size - 1);
    double angle_rad = angle_deg * M_PI / 180.0;

    persistent_x[persistent_points] = distance * cos(angle_rad);
    persistent_y[persistent_points] = -distance * sin(angle_rad);
    persistent_points++;
}

static void clear_pending_scan(void)
{
    for (int i = 0; i < LD06_SCAN_SIZE; i++) {
        sensor_data.d[i] = -1;
        scan_received[i] = 0;
    }
    scan_points_received = 0;
}

static int serial_init(const char *port_name)
{
    int serial_port = open(port_name, O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (serial_port < 0) {
        perror("Could not open serial port");
        return -1;
    }

    struct termios tty;
    if (tcgetattr(serial_port, &tty) != 0) {
        perror("Could not read serial settings");
        close(serial_port);
        return -1;
    }

    cfsetispeed(&tty, SERIAL_BAUD);
    cfsetospeed(&tty, SERIAL_BAUD);

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

static int serial_read_line(int serial_port, char line[], int line_size)
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
        line[position++] = character;
    } else {
        position = 0;
    }

    return 2;
}

static int angle_to_bucket(float angle)
{
    while (angle < 0.0f) {
        angle += 360.0f;
    }
    while (angle >= 360.0f) {
        angle -= 360.0f;
    }

    int bucket = (int)((angle / 360.0f) * LD06_SCAN_SIZE);
    if (bucket >= LD06_SCAN_SIZE) {
        bucket = LD06_SCAN_SIZE - 1;
    }
    return bucket;
}

static void process_line(const char *line)
{
    int point_number;
    float angle;
    int distance;
    int intensity;
    int q1;
    int q2;
    unsigned long timestamp;

    if (line[0] == '\0') {
        if (scan_points_received > 0) {
            ts_build_scan(&sensor_data, &latest_scan, &state, 1);
            scan_count++;
        }
        clear_pending_scan();
        return;
    }

    if (sscanf(line, "time: %lu", &timestamp) == 1) {
        sensor_data.timestamp = (unsigned int)timestamp;
        return;
    }

    if (sscanf(line, "Index:%d:%d", &q1, &q2) == 2) {
        sensor_data.q1 = q1;
        sensor_data.q2 = q2;
        return;
    }

    if (sscanf(line, "%d,%f,%d,%d", &point_number, &angle, &distance, &intensity) == 4) {
        int bucket = angle_to_bucket(angle);
        sensor_data.d[bucket] = distance;
        lidar_points_parsed++;
        remember_point_from_bucket(bucket, distance);
        if (!scan_received[bucket]) {
            scan_received[bucket] = 1;
            scan_points_received++;
        }
        ts_build_scan(&sensor_data, &latest_scan, &state, 1);
    }
}

static Vector2 scan_to_screen(Vector2 center, float mm_to_px, double x, double y)
{
    return (Vector2){
        center.x + (float)x * mm_to_px,
        center.y - (float)y * mm_to_px
    };
}

static Color point_color(int index, int total)
{
    if (total <= 1) {
        return BLUE;
    }

    float t = (float)index / (float)(total - 1);
    unsigned char r = (unsigned char)(40 + 215 * t);
    unsigned char b = (unsigned char)(255 - 215 * t);
    return (Color){r, 80, b, 255};
}

static void draw_scan_view(const char *port_name, int serial_ok)
{
    Vector2 center = {(float)GetScreenWidth() * 0.5f, (float)GetScreenHeight() * 0.54f};
    float mm_to_px = 0.075f;
    float radius_mm = 6000.0f;

    DrawCircleLines((int)center.x, (int)center.y, radius_mm * mm_to_px, Fade(DARKGRAY, 0.35f));
    DrawCircleLines((int)center.x, (int)center.y, 3000.0f * mm_to_px, Fade(DARKGRAY, 0.22f));

    Vector2 x_axis = scan_to_screen(center, mm_to_px, 6200, 0);
    Vector2 y_axis = scan_to_screen(center, mm_to_px, 0, 6200);
    Vector2 neg_x_axis = scan_to_screen(center, mm_to_px, -6200, 0);
    Vector2 neg_y_axis = scan_to_screen(center, mm_to_px, 0, -6200);

    DrawLineEx(neg_x_axis, x_axis, 2.0f, BLACK);
    DrawLineEx(neg_y_axis, y_axis, 2.0f, BLACK);
    DrawText("+X / 0 deg", (int)x_axis.x - 70, (int)x_axis.y + 8, 18, MAROON);
    DrawText("-X / 180 deg", (int)neg_x_axis.x + 8, (int)neg_x_axis.y + 8, 18, MAROON);
    DrawText("+Y / 90 deg", (int)y_axis.x + 8, (int)y_axis.y - 24, 18, DARKGREEN);
    DrawText("-Y / 270 deg", (int)neg_y_axis.x + 8, (int)neg_y_axis.y + 8, 18, DARKGREEN);

    DrawCircleV(center, 5.0f, BLACK);
    DrawTriangle(
        (Vector2){center.x + 20, center.y},
        (Vector2){center.x - 12, center.y - 10},
        (Vector2){center.x - 12, center.y + 10},
        Fade(BLACK, 0.85f));

    for (int i = 0; i < persistent_points; i++) {
        Vector2 p = scan_to_screen(center, mm_to_px, persistent_x[i], persistent_y[i]);
        DrawPixelV(p, Fade(BLACK, 0.28f));
    }

    for (int i = 0; i < latest_scan.nb_points; i++) {
        Vector2 p = scan_to_screen(center, mm_to_px, latest_scan.x[i], latest_scan.y[i]);
        DrawCircleV(p, 2.0f, point_color(i, latest_scan.nb_points));
    }

    if (latest_scan.nb_points > 1) {
        Vector2 first = scan_to_screen(center, mm_to_px, latest_scan.x[0], latest_scan.y[0]);
        Vector2 last = scan_to_screen(center, mm_to_px,
            latest_scan.x[latest_scan.nb_points - 1],
            latest_scan.y[latest_scan.nb_points - 1]);
        DrawCircleV(first, 7.0f, BLUE);
        DrawCircleV(last, 7.0f, RED);
        DrawText("first", (int)first.x + 8, (int)first.y - 8, 16, BLUE);
        DrawText("last", (int)last.x + 8, (int)last.y - 8, 16, RED);
    }

    DrawText("CoreSLAM local lidar frame", 24, 24, 28, BLACK);
    DrawText(TextFormat("port: %s", port_name), 24, 65, 18, DARKGRAY);
    DrawText(TextFormat("serial: %s", serial_ok ? "open" : "not open"), 24, 92, 18, serial_ok ? DARKGREEN : RED);
    DrawText(TextFormat("scan: %d", scan_count), 24, 119, 18, DARKGRAY);
    DrawText(TextFormat("points in latest ts_build_scan: %d", latest_scan.nb_points), 24, 146, 18, DARKGRAY);
    DrawText(TextFormat("serial lines received: %d", lines_received), 24, 173, 18, DARKGRAY);
    DrawText(TextFormat("lidar CSV points parsed: %d", lidar_points_parsed), 24, 200, 18, DARKGRAY);
    DrawText(TextFormat("current buckets received: %d", scan_points_received), 24, 227, 18, DARKGRAY);
    DrawText(TextFormat("persistent plotted points: %d", persistent_points), 24, 254, 18, DARKGRAY);
    DrawText("Blue = earliest built point, red = latest built point", 24, GetScreenHeight() - 50, 18, DARKGRAY);
}

int main(int argc, char *argv[])
{
    const char *port_name = (argc > 1) ? argv[1] : DEFAULT_SERIAL_PORT;

    ts_map_init(&dummy_map);
    ts_state_init(&state, &dummy_map, &params, &laser_params,
                  &start_position, 0, 0, 300, TS_DIRECTION_FORWARD);
    clear_pending_scan();

    int serial_port = serial_init(port_name);
    int serial_ok = serial_port >= 0;
    char line[LINE_SIZE];

    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(WINDOW_W, WINDOW_H, "LiDAR Local Scan Debug Viewer");
    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        if (serial_ok) {
            int lines_processed = 0;
            while (lines_processed < 4096) {
                int line_received = serial_read_line(serial_port, line, LINE_SIZE);
                if (line_received < 0) {
                    close(serial_port);
                    serial_port = -1;
                    serial_ok = 0;
                    break;
                }
                if (line_received == 0) {
                    break;
                }
                if (line_received == 1) {
                    lines_received++;
                    process_line(line);
                }
                lines_processed++;
            }
        }

        BeginDrawing();
        ClearBackground(RAYWHITE);
        draw_scan_view(port_name, serial_ok);
        EndDrawing();
    }

    if (serial_port >= 0) {
        close(serial_port);
    }
    CloseWindow();
    return 0;
}
