#include "raylib.h"

#include <math.h>
#include <stdio.h>

#include "CoreSLAM.h"
#include "output_manager.h"

#define WINDOW_W 1100
#define WINDOW_H 950
#define AXIS_MARGIN 70
#define RIGHT_MARGIN 260
#define TOP_MARGIN 20
#define BOTTOM_MARGIN 60
#define DEFAULT_SERIAL_PORT "/dev/ttyACM0"
#define MAJOR_GRID_STEP_CELLS 128

int slam_controller_init(const char *port_name);
void slam_controller_update(void);
void slam_controller_close(void);



// gcc raylib_map_viewer.c Slam_Controller.c CoreSLAM_state.c CoreSLAM_random.c CoreSLAM.c CoreSLAM_ext.c CoreSLAM_loop_closing.c output_manager.c -o slam_viewer -lraylib -lGL -lm -lpthread -ldl -lrt -lX11
// ./slam_viewer

// usbipd list
// usbipd bind --busid 1-3
// usbipd attach --wsl --busid 1-3


static Color map_pixels[TS_MAP_SIZE * TS_MAP_SIZE];

static float grid_left;
static float grid_top;
static float grid_size_px;
static float cell_px;

static void update_layout(void)
{
    int available_w = GetScreenWidth() - AXIS_MARGIN - RIGHT_MARGIN;
    int available_h = GetScreenHeight() - TOP_MARGIN - BOTTOM_MARGIN;

    if (available_w < 100) available_w = 100;
    if (available_h < 100) available_h = 100;

    grid_size_px = (available_w < available_h) ? (float)available_w : (float)available_h;
    cell_px = grid_size_px / (float)TS_MAP_SIZE;
    grid_left = AXIS_MARGIN;
    grid_top = TOP_MARGIN;
}

static Color map_value_to_color(ts_map_pixel_t value)
{
    if (value == TS_OBSTACLE) {
        return BLACK;
    }

    if (value >= TS_NO_OBSTACLE) {
        return WHITE;
    }

    unsigned char c = (unsigned char)(((unsigned int)value * 255u) / TS_NO_OBSTACLE);
    return (Color){ c, c, c, 255 };
}

static void update_map_pixels(const ts_map_t *map)
{
    for (int y = 0; y < TS_MAP_SIZE; y++) {
        int screen_y = TS_MAP_SIZE - 1 - y;

        for (int x = 0; x < TS_MAP_SIZE; x++) {
            map_pixels[screen_y * TS_MAP_SIZE + x] =
                map_value_to_color(map->map[y * TS_MAP_SIZE + x]);
        }
    }
}

static Vector2 map_cell_to_screen(float cell_x, float cell_y)
{
    return (Vector2){
        grid_left + cell_x * cell_px,
        grid_top + ((float)TS_MAP_SIZE - 1.0f - cell_y) * cell_px
    };
}

static void draw_axes_and_grid(void)
{
    float left = grid_left;
    float right = grid_left + grid_size_px;
    float top = grid_top;
    float bottom = grid_top + grid_size_px;

    DrawRectangleLines((int)left, (int)top, (int)grid_size_px, (int)grid_size_px, BLACK);

    for (int cell = 0; cell <= TS_MAP_SIZE; cell += MAJOR_GRID_STEP_CELLS) {
        float x = left + cell * cell_px;
        float y = bottom - cell * cell_px;
        int mm = (int)((double)cell / TS_MAP_SCALE);

        DrawLine((int)x, (int)top, (int)x, (int)bottom, Fade(BLACK, 0.18f));
        DrawLine((int)left, (int)y, (int)right, (int)y, Fade(BLACK, 0.18f));

        DrawText(TextFormat("%d", mm), (int)(x - 18), (int)(bottom + 8), 12, BLACK);
        DrawText(TextFormat("%d", mm), 8, (int)(y - 6), 12, BLACK);
    }

    DrawLineEx((Vector2){ left, bottom }, (Vector2){ right, bottom }, 2.0f, BLACK);
    DrawLineEx((Vector2){ left, bottom }, (Vector2){ left, top }, 2.0f, BLACK);
    DrawText("X mm", (int)(right - 42), (int)(bottom + 30), 14, BLACK);
    DrawTextPro(GetFontDefault(), "Y mm", (Vector2){ 24, top + 42 }, (Vector2){ 0, 0 }, -90.0f, 14, 1.0f, BLACK);
}

static void draw_robot(ts_position_t position)
{
    float cell_x = (float)(position.x * TS_MAP_SCALE);
    float cell_y = (float)(position.y * TS_MAP_SCALE);

    if (cell_x < 0 || cell_x >= TS_MAP_SIZE || cell_y < 0 || cell_y >= TS_MAP_SIZE) {
        DrawText("Robot outside map", GetScreenWidth() - RIGHT_MARGIN + 20, 130, 18, RED);
        return;
    }

    Vector2 center = map_cell_to_screen(cell_x, cell_y);
    float theta = (float)(position.theta * M_PI / 180.0);
    Vector2 forward = { cosf(theta), -sinf(theta) };
    Vector2 right = { -forward.y, forward.x };
    float size = 16.0f;

    Vector2 nose = {
        center.x + forward.x * size,
        center.y + forward.y * size
    };
    Vector2 back_left = {
        center.x - forward.x * size * 0.70f + right.x * size * 0.55f,
        center.y - forward.y * size * 0.70f + right.y * size * 0.55f
    };
    Vector2 back_right = {
        center.x - forward.x * size * 0.70f - right.x * size * 0.55f,
        center.y - forward.y * size * 0.70f - right.y * size * 0.55f
    };

    DrawTriangle(nose, back_left, back_right, RED);
    DrawTriangleLines(nose, back_left, back_right, BLACK);
    DrawCircleV(center, 3.5f, BLACK);
}

static void draw_status_panel(ts_position_t position, const char *port_name)
{
    int x = GetScreenWidth() - RIGHT_MARGIN + 20;

    DrawText("SLAM", x, 30, 24, BLACK);
    DrawText(TextFormat("port: %s", port_name), x, 70, 16, DARKGRAY);
    DrawText(TextFormat("grid: %d x %d", TS_MAP_SIZE, TS_MAP_SIZE), x, 100, 16, DARKGRAY);
    DrawText(TextFormat("cell: %.1f mm", 1.0 / TS_MAP_SCALE), x, 130, 16, DARKGRAY);

    DrawText("robot", x, 180, 20, BLACK);
    DrawText(TextFormat("x: %.2f mm", position.x), x, 215, 16, BLACK);
    DrawText(TextFormat("y: %.2f mm", position.y), x, 245, 16, BLACK);
    DrawText(TextFormat("theta: %.2f deg", position.theta), x, 275, 16, BLACK);
}

int main(int argc, char *argv[])
{
    const char *port_name = (argc > 1) ? argv[1] : DEFAULT_SERIAL_PORT;

    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(WINDOW_W, WINDOW_H, "CoreSLAM Raylib Viewer");
    SetTargetFPS(30);

    int serial_ok = slam_controller_init(port_name);
    if (!serial_ok) {
        fprintf(stderr, "Viewer opened, but SLAM serial did not start.\n");
    }

    for (int i = 0; i < TS_MAP_SIZE * TS_MAP_SIZE; i++) {
        map_pixels[i] = (Color){ 128, 128, 128, 255 };
    }

    Image map_image = {
        .data = map_pixels,
        .width = TS_MAP_SIZE,
        .height = TS_MAP_SIZE,
        .mipmaps = 1,
        .format = PIXELFORMAT_UNCOMPRESSED_R8G8B8A8
    };

    Texture2D map_texture = LoadTextureFromImage(map_image);
    SetTextureFilter(map_texture, TEXTURE_FILTER_POINT);

    while (!WindowShouldClose()) {
        slam_controller_update();

        ts_map_t *map = get_map();
        ts_position_t position = get_position();

        if (map != 0) {
            update_map_pixels(map);
            UpdateTexture(map_texture, map_pixels);
        }

        update_layout();

        BeginDrawing();
        ClearBackground(RAYWHITE);

        Rectangle src = { 0, 0, (float)TS_MAP_SIZE, (float)TS_MAP_SIZE };
        Rectangle dst = { grid_left, grid_top, grid_size_px, grid_size_px };
        DrawTexturePro(map_texture, src, dst, (Vector2){ 0, 0 }, 0.0f, WHITE);

        draw_axes_and_grid();
        draw_robot(position);
        draw_status_panel(position, port_name);

        if (!serial_ok) {
            DrawText("NO SERIAL", GetScreenWidth() - RIGHT_MARGIN + 20, 330, 20, RED);
        }

        EndDrawing();
    }

    UnloadTexture(map_texture);
    slam_controller_close();
    CloseWindow();
    return 0;
}
