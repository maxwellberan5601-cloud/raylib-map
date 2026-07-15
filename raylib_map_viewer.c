#include "raylib.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>

/* ---------------- CONFIG ---------------- */

/* Must match MAP_SIZE on the Arduino side. Array is GRID_SIZE*GRID_SIZE. */
#define GRID_SIZE 16

/* Real-world size of one cell (base = width, height = height) */
#define CELL_SIZE_MM 10

#define WINDOW_W 900
#define WINDOW_H 900

/* Margin reserved for axis lines/labels */
#define AXIS_MARGIN 60
#define RIGHT_MARGIN 20
#define TOP_MARGIN 20

/* Below this pixel-per-cell size, switch to the fast heatmap texture */
#define MIN_CELL_PX_FOR_LABELS 18

/* Skip axis tick labels if there'd be more than this many */
#define MAX_AXIS_TICK_LABELS 40

#define DEFAULT_SERIAL_PORT "/dev/ttyACM0"
#define SERIAL_BAUD B115200

/* ----------------------------------------- */

#define CELL_COUNT (GRID_SIZE * GRID_SIZE)

static int g_map[CELL_COUNT];      /* live grid values, index = gridRow*GRID_SIZE+col (row 0 = bottom) */
static int g_minValue = 0;
static int g_maxValue = 1;
static unsigned long g_receivedCount = 0; /* total values ever received */

static float g_gridAreaW;
static float g_gridAreaH;
static float g_cellPxW;
static float g_cellPxH;

/* ---------------- Serial handling ---------------- */

static int g_serialFd = -1;
/* A full "MAP:(i,val),..." line for GRID_SIZE=16 (256 pairs) can run to a
 * few KB, so this needs to be much bigger than a single-number line. */
static char g_lineBuf[8192];
static int g_lineLen = 0;
static unsigned long g_frameCount = 0; /* number of full MAP: lines received */

static int OpenSerialPort(const char *device)
{
    int fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0)
    {
        fprintf(stderr, "Could not open serial port '%s': %s\n", device, strerror(errno));
        fprintf(stderr, "Check the device path and permissions (try `sudo usermod -aG dialout $USER` then re-login).\n");
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0)
    {
        fprintf(stderr, "tcgetattr failed on '%s': %s\n", device, strerror(errno));
        close(fd);
        return -1;
    }

    cfsetispeed(&tty, SERIAL_BAUD);
    cfsetospeed(&tty, SERIAL_BAUD);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; /* 8 data bits */
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;   /* no canonical mode, no echo */
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0; /* non-blocking read */
    tty.c_cc[VTIME] = 0;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); /* no software flow control */
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD); /* no parity */
    tty.c_cflag &= ~CSTOPB;            /* 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;           /* no hardware flow control */

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        fprintf(stderr, "tcsetattr failed on '%s': %s\n", device, strerror(errno));
        close(fd);
        return -1;
    }

    return fd;
}

/* Store one (index, value) pair straight into its grid slot.
 * index 0 = bottom-left ... index CELL_COUNT-1 = top-right, same layout
 * the sender already uses -- so no reordering needed, just bounds-check. */
static void StoreAtIndex(int idx, int value)
{
    if (idx < 0 || idx >= CELL_COUNT) return; /* ignore out-of-range indices */
    g_map[idx] = value;
    g_receivedCount++;
}

/* Parse one full grid line of the form:
 *   MAP:(0,0),(1,489),(2,978),...,(255,124695)
 * Every "(i,val)" pair found is stored at g_map[i] = val. Malformed pairs
 * are skipped (so one bad pair doesn't throw off the rest of the line). */
static void ParseMapLine(const char *body)
{
    const char *p = body;
    int pairsParsed = 0;

    while (*p != '\0')
    {
        /* skip anything that isn't the start of a pair */
        while (*p != '\0' && *p != '(') p++;
        if (*p == '\0') break;

        int idx, val;
        int consumed = 0;
        if (sscanf(p, "(%d,%d)%n", &idx, &val, &consumed) == 2)
        {
            StoreAtIndex(idx, val);
            pairsParsed++;
            p += consumed;
        }
        else
        {
            p++; /* couldn't parse a pair here -- advance one char and keep scanning */
        }
    }

    if (pairsParsed > 0) g_frameCount++;
}

/* Parse a line; only accept lines that either:
 *   - start with "MAP:" followed by (i,val) pairs, or
 *   - are (kept for backward compatibility) a single clean integer.
 * Anything else (blank lines, debug text, stray LiDAR prints, etc.) is
 * ignored. */
static void ProcessLine(const char *rawLine)
{
    const char *p = rawLine;
    while (*p == ' ' || *p == '\t') p++;
    if (*p == '\0') return;

    if (strncmp(p, "MAP:", 4) == 0)
    {
        ParseMapLine(p + 4);
        return;
    }

    char *endptr = NULL;
    long val = strtol(p, &endptr, 10);
    if (endptr == p) return; /* no digits parsed at all -- not a value line */

    while (*endptr == ' ' || *endptr == '\t' || *endptr == '\r') endptr++;
    if (*endptr != '\0') return; /* trailing junk -> ignore */

    /* Fallback for the old one-value-per-line format: cycle-fill like before */
    unsigned long idx = g_receivedCount % (unsigned long)CELL_COUNT;
    StoreAtIndex((int)idx, (int)val);
}

/* Drain whatever bytes are currently available without blocking, split into
 * lines, and process each complete line. Call once per frame. */
static void PollSerial(void)
{
    if (g_serialFd < 0) return;

    char chunk[256];
    ssize_t n;
    while ((n = read(g_serialFd, chunk, sizeof(chunk))) > 0)
    {
        for (ssize_t i = 0; i < n; i++)
        {
            char c = chunk[i];
            if (c == '\n' || c == '\r')
            {
                if (g_lineLen > 0)
                {
                    g_lineBuf[g_lineLen] = '\0';
                    ProcessLine(g_lineBuf);
                    g_lineLen = 0;
                }
            }
            else if (g_lineLen < (int)sizeof(g_lineBuf) - 1)
            {
                g_lineBuf[g_lineLen++] = c;
            }
        }
    }
}

/* ---------------- Grid helpers ---------------- */

static void UpdateGridLayout(void)
{
    int screenW = GetScreenWidth();
    int screenH = GetScreenHeight();

    int availableW = screenW - AXIS_MARGIN - RIGHT_MARGIN;
    int availableH = screenH - AXIS_MARGIN - TOP_MARGIN;

    if (availableW < GRID_SIZE) availableW = GRID_SIZE;
    if (availableH < GRID_SIZE) availableH = GRID_SIZE;

    /* Keep cells square so a 10mm x 10mm cell doesn't turn rectangular
     * when the window is resized to a non-square shape. */
    float gridSide = (availableW < availableH) ? (float)availableW : (float)availableH;

    g_gridAreaW = gridSide;
    g_gridAreaH = gridSide;
    g_cellPxW = g_gridAreaW / (float)GRID_SIZE;
    g_cellPxH = g_gridAreaH / (float)GRID_SIZE;
}

/* Recompute min/max across the CURRENT grid contents, for auto color scaling */
static void RecomputeMinMax(void)
{
    int mn = g_map[0];
    int mx = g_map[0];
    for (int i = 1; i < CELL_COUNT; i++)
    {
        if (g_map[i] < mn) mn = g_map[i];
        if (g_map[i] > mx) mx = g_map[i];
    }
    if (mx == mn) mx = mn + 1; /* avoid divide-by-zero when everything's equal */
    g_minValue = mn;
    g_maxValue = mx;
}

/* Map current value to grayscale using the live min(-> black)/max(-> white) */
static Color ValueToColor(int value)
{
    float t = (float)(value - g_minValue) / (float)(g_maxValue - g_minValue);
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    unsigned char c = (unsigned char)(t * 255.0f);
    Color col = { c, c, c, 255 };
    return col;
}

/* Convert a grid row (0 = bottom) into a screen row (0 = top). */
static int GridRowToScreenRow(int gridRow)
{
    return (GRID_SIZE - 1) - gridRow;
}

/* ---------- Axis drawing: origin (0,0) at bottom-left of the grid ---------- */
static void DrawAxes(void)
{
    int screenH = GetScreenHeight();

    float originX = AXIS_MARGIN;
    float originY = screenH - AXIS_MARGIN;
    float topY    = originY - g_gridAreaH;
    float rightX  = AXIS_MARGIN + g_gridAreaW;

    DrawLineEx((Vector2){ originX, originY }, (Vector2){ originX, topY }, 2.0f, BLACK);
    DrawLineEx((Vector2){ originX, originY }, (Vector2){ rightX, originY }, 2.0f, BLACK);

    int totalTicks = GRID_SIZE + 1;
    int step = 1;
    while ((totalTicks / step) > MAX_AXIS_TICK_LABELS) step *= 2;

    for (int i = 0; i <= GRID_SIZE; i += step)
    {
        int mmValue = i * CELL_SIZE_MM;

        float tx = originX + i * g_cellPxW;
        DrawLineEx((Vector2){ tx, originY }, (Vector2){ tx, originY + 6 }, 1.0f, BLACK);
        char bufX[16];
        snprintf(bufX, sizeof(bufX), "%d", mmValue);
        int wX = MeasureText(bufX, 12);
        DrawText(bufX, (int)(tx - wX / 2.0f), (int)(originY + 10), 12, BLACK);

        float ty = originY - i * g_cellPxH;
        DrawLineEx((Vector2){ originX - 6, ty }, (Vector2){ originX, ty }, 1.0f, BLACK);
        char bufY[16];
        snprintf(bufY, sizeof(bufY), "%d", mmValue);
        int wY = MeasureText(bufY, 12);
        DrawText(bufY, (int)(originX - 12 - wY), (int)(ty - 6), 12, BLACK);
    }

    DrawText("X (mm)", (int)(rightX - 50), (int)(originY + 28), 14, BLACK);
    DrawTextPro(GetFontDefault(), "Y (mm)", (Vector2){ 18, originY - 40 }, (Vector2){ 0, 0 }, -90.0f, 14, 1.0f, BLACK);

    DrawCircle((int)originX, (int)originY, 3, RED);
    DrawText("0", (int)(originX - 14), (int)(originY + 8), 12, RED);
}

/* ---------- Rendering path 1: labeled cells (small/medium grids) ---------- */
static void DrawLabeledGrid(void)
{
    for (int gridRow = 0; gridRow < GRID_SIZE; gridRow++)
    {
        int screenRow = GridRowToScreenRow(gridRow);
        for (int col = 0; col < GRID_SIZE; col++)
        {
            int value = g_map[gridRow * GRID_SIZE + col];
            Color fill = ValueToColor(value);

            float x = AXIS_MARGIN + col * g_cellPxW;
            float y = (GetScreenHeight() - AXIS_MARGIN - g_gridAreaH) + screenRow * g_cellPxH;

            DrawRectangle((int)x, (int)y, (int)ceilf(g_cellPxW), (int)ceilf(g_cellPxH), fill);
            DrawRectangleLines((int)x, (int)y, (int)g_cellPxW, (int)g_cellPxH, BLACK);

            Color textColor = (value > (g_minValue + g_maxValue) / 2) ? BLACK : WHITE;

            char buf[16];
            snprintf(buf, sizeof(buf), "%d", value);
            int fontSize = (int)(g_cellPxH * 0.35f);
            if (fontSize < 8) fontSize = 8;
            int textW = MeasureText(buf, fontSize);

            DrawText(buf,
                     (int)(x + (g_cellPxW - textW) / 2.0f),
                     (int)(y + (g_cellPxH - fontSize) / 2.0f),
                     fontSize, textColor);
        }
    }
}

/* ---------- Rendering path 2: fast heatmap texture (large grids) ---------- */
/* Rebuilt every frame from live g_map data, since values keep changing. */
static void UpdateHeatmapImage(Color *pixels)
{
    for (int gridRow = 0; gridRow < GRID_SIZE; gridRow++)
    {
        int screenRow = GridRowToScreenRow(gridRow);
        for (int col = 0; col < GRID_SIZE; col++)
        {
            int value = g_map[gridRow * GRID_SIZE + col];
            pixels[screenRow * GRID_SIZE + col] = ValueToColor(value);
        }
    }
}

int main(int argc, char *argv[])
{
    const char *portPath = (argc > 1) ? argv[1] : DEFAULT_SERIAL_PORT;
    g_serialFd = OpenSerialPort(portPath);
    if (g_serialFd < 0)
    {
        fprintf(stderr, "Continuing without live data (grid will show zeros).\n");
    }
    else
    {
        printf("Reading live values from %s\n", portPath);
    }

    memset(g_map, 0, sizeof(g_map));

    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(WINDOW_W, WINDOW_H, "Grid Map Viewer - Live Serial (mm axes)");
    SetTargetFPS(60);

    static Color heatPixels[CELL_COUNT];
    Image heatImg = { .data = heatPixels, .width = GRID_SIZE, .height = GRID_SIZE,
                       .mipmaps = 1, .format = PIXELFORMAT_UNCOMPRESSED_R8G8B8A8 };
    Texture2D heatTex = LoadTextureFromImage(heatImg);
    SetTextureFilter(heatTex, TEXTURE_FILTER_POINT);

    while (!WindowShouldClose())
    {
        PollSerial();
        RecomputeMinMax();
        UpdateGridLayout();
        bool useLabels = (g_cellPxW >= MIN_CELL_PX_FOR_LABELS && g_cellPxH >= MIN_CELL_PX_FOR_LABELS);

        BeginDrawing();
        ClearBackground(RAYWHITE);

        if (useLabels)
        {
            DrawLabeledGrid();
        }
        else
        {
            UpdateHeatmapImage(heatPixels);
            UpdateTexture(heatTex, heatPixels);

            Rectangle src = { 0, 0, (float)heatTex.width, (float)heatTex.height };
            Rectangle dst = { AXIS_MARGIN, GetScreenHeight() - AXIS_MARGIN - g_gridAreaH, g_gridAreaW, g_gridAreaH };
            DrawTexturePro(heatTex, src, dst, (Vector2){ 0, 0 }, 0.0f, WHITE);

            DrawText(TextFormat("GRID_SIZE=%d too dense for per-cell borders/labels - showing heatmap", GRID_SIZE),
                      AXIS_MARGIN, 10, 14, RED);
        }

        DrawAxes();

        const char *status = (g_serialFd < 0) ? "NO SERIAL CONNECTION" : "LIVE";
        DrawText(TextFormat("[%s] Each cell = %d mm x %d mm. Grids received: %lu | shown range: %d (black) - %d (white)",
                             status, CELL_SIZE_MM, CELL_SIZE_MM, g_frameCount, g_minValue, g_maxValue),
                  AXIS_MARGIN, GetScreenHeight() - 20, 14, DARKGRAY);

        EndDrawing();
    }

    if (g_serialFd >= 0) close(g_serialFd);
    UnloadTexture(heatTex);
    CloseWindow();

    return 0;
}