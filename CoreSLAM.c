#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#ifdef _MSC_VER
   typedef __int64 int64_t;	// Define it from MSVC's internal type
#else
   #include <stdint.h>		// Use the C99 official header
#endif
#include "CoreSLAM.h"

void ts_map_init(ts_map_t *map) //go through every uint16_t in the map array and set them init value = unkown 
{
    int x, y, initval; //initval value used to initalise each cell
    ts_map_pixel_t *ptr;
    initval = (TS_OBSTACLE + TS_NO_OBSTACLE) / 2; //which is value for unkown
    for (ptr = map->map, y = 0; y < TS_MAP_SIZE; y++) {
	for (x = 0; x < TS_MAP_SIZE; x++, ptr++) {
	    *ptr = initval;
	}
    }
}

//find global x and y
//scan match cost
//pos is candiate postion
int ts_distance_scan_to_map(ts_scan_t *scan, ts_map_t *map, ts_position_t *pos)
{
    double c, s;
    int i, x, y, nb_points = 0; //nb_point
    int64_t sum;

    c = cos(pos->theta * M_PI / 180);
    s = sin(pos->theta * M_PI / 180);
    // Translate and rotate scan to robot position
    // and compute the distance

    //if the scans a vlid number
    for (i = 0, sum = 0; i != scan->nb_points; i++) {
        if (scan->value[i] != TS_NO_OBSTACLE) {
            x = (int)floor((pos->x + c * scan->x[i] - s * scan->y[i]) * TS_MAP_SCALE + 0.5); //0.5 makes floor round to higher int //find global x
            y = (int)floor((pos->y + s*scan->x[i] + c*scan->y[i])* TS_MAP_SCALE + 0.5);
            // Check boundaries
            if (x >= 0 && x < TS_MAP_SIZE && y >= 0 && y < TS_MAP_SIZE) {
                sum += map->map[y * TS_MAP_SIZE + x]; //search the point on the map using the global x, y we found in the scan
                //the values added sum will be low as all x, y should be obstacles = 0 so if there higher match is off and theres a cost assigned
                nb_points++;
            }
        }
    }
    if (nb_points) //if there is atleast one valid nb_point (boolean any non-zero is true)
        {sum = sum * 1024 / nb_points;}
    else sum = 2000000000; //if no scan points are valid give huge penality
    return (int)sum;
}

#define SWAP(x, y) (x ^= y ^= x ^= y) //it makes a kind of function that swaps the varable values you put into it

void ts_map_laser_ray(ts_map_t *map, int x1, int y1, int x2, int y2, 
                 int xp, int yp, int value, int alpha)

{
    //x1,y1 where the lidar is
    //xp, yp, a point the lidar hit
    //x2,y2 slighlty beyond where the lidar hit to take into consideration  the obstacles are thick
    //the value of free unkown or full
    //aplha how strongly this changes the map
    int x2c, y2c, dx, dy, dxc, dyc, error, errorv, derrorv, x;
    //x2c, y2c, clipped x and y
    //dx, dy full ray displacement, dxc, dyc clipped ray displacement
    //bresenham line-postion error, errorv occupancy interpolation remainder
    //derrorv length of obstacle-transistion section
    //x loop counter along domaint ray axis
    int incv, sincv, incerrorv, incptrx, incptry, pixval, horiz, diago;
    //incv change in occupancy value per step, sinv sign correction either 1 or 0
    //incerrorv remainder interger occupancy interpolation, incptrx memory step along secondary direction
    //incptry memory step along secondary direction

    ts_map_pixel_t *ptr;

    //is lidar outside the map
    if (x1 < 0 || x1 >= TS_MAP_SIZE || y1 < 0 || y1 >= TS_MAP_SIZE)
        return; // Robot is out of map
        //when this happen robot doesn't crash 
        //simply the grid is never updates it just exits the function
    
    x2c = x2; y2c = y2;
    // Clipping
    //clip x2c and y2c on the map boarder
    if (x2c < 0) {
        if (x2c == x1) return; //exit the function to prevent divison error just in case
        y2c += (y2c - y1) * (-x2c) / (x2c - x1); //y_crosses = -mx + y2c where x=x2c then subtract from 
        x2c = 0;
    }
    if (x2c >= TS_MAP_SIZE) {
        if (x1 == x2c) return;
        y2c += (y2c - y1) * (TS_MAP_SIZE - 1 - x2c) / (x2c - x1);
        x2c = TS_MAP_SIZE - 1;
    }
    if (y2c < 0) {
        if (y1 == y2c) return;
        x2c += (x1 - x2c) * (-y2c) / (y1 - y2c);
        y2c = 0;
    }
    if (y2c >= TS_MAP_SIZE) {
        if (y1 == y2c) return;
        x2c += (x1 - x2c) * (TS_MAP_SIZE - 1 - y2c) / (y1 - y2c);
        y2c = TS_MAP_SIZE - 1;
    }

    //ray distances
    dx = abs(x2 - x1); dy = abs(y2 - y1);
    //clipped by boarder ray distances
    dxc = abs(x2c - x1); dyc = abs(y2c - y1);

    //ray single step directions
    incptrx = (x2 > x1) ? 1 : -1; //are the rays moving left or right
    // the if statement equvalient of this tenary operator is//if (x2 > x1)
                                                            //     incptrx = 1;
                                                            // else
                                                            //     incptrx = -1;
    incptry = (y2 > y1) ? TS_MAP_SIZE : -TS_MAP_SIZE; //because adding TS_MAP_SIZE to indice is the equalvient moving up or 
    //down a column becaus map is 1D array
    //ray single step directions

    //decides when occpuancy grid value is corrected what direction will it be corrected
    sincv = (value > TS_NO_OBSTACLE) ? 1 : -1; 

    //is x or y the domaint axis
    if (dx > dy) {
        derrorv = abs(xp - x2); //length of domanit axis controls width of occupany transistion
    } else {
        //swaps dx with dy so when the rest programs needs domaint axis it will be dy 
        SWAP(dx, dy); SWAP(dxc, dyc); SWAP(incptrx, incptry);        
        derrorv = abs(yp - y2);
    }

    //Initialise Bresenham’s line error
    //error decides when will the line step along the secondary axis
    error = 2 * dyc - dxc;
    horiz = 2 * dyc;
    diago = 2 * (dyc - dxc);
    errorv = derrorv / 2;
    incv = (value - TS_NO_OBSTACLE) / derrorv;
    incerrorv = value - TS_NO_OBSTACLE - derrorv * incv;  
    ptr = map->map + y1 * TS_MAP_SIZE + x1;
    pixval = TS_NO_OBSTACLE;
    for (x = 0; x <= dxc; x++, ptr += incptrx) {
        if (x > dx - 2 * derrorv) {
            if (x <= dx - derrorv) {
                pixval += incv;
                errorv += incerrorv;
                if (errorv > derrorv) {
                    pixval += sincv;
                    errorv -= derrorv; 
                }
            } else {
                pixval -= incv;
                errorv -= incerrorv;
                if (errorv < 0) {
                    pixval -= sincv;
                    errorv += derrorv; 
                }
            }
        } 
        //Bresenham’s line
        // Integration into the map
        *ptr = ((256 - alpha) * (*ptr) + alpha * pixval) >> 8;  	
        if (error > 0) {
            ptr += incptry;
            error += diago;
        } else error += horiz;
    }
}

void ts_map_update(ts_scan_t *scan, ts_map_t *map, ts_position_t *pos, int quality, int hole_width)
{
    double c, s;
    double x2p, y2p;
    int i, x1, y1, x2, y2, xp, yp, value, q;
    double add, dist;

    c = cos(pos->theta * M_PI / 180);
    s = sin(pos->theta * M_PI / 180);
    //convert x y cordinates to grid cells
    x1 = (int)floor(pos->x * TS_MAP_SCALE + 0.5); //TS_MAP_SCALE = 1/10 where 10 is in mm, 0.5 so it rounds to higher int
    y1 = (int)floor(pos->y * TS_MAP_SCALE + 0.5);
    // Translate and rotate scan to robot position
    for (i = 0; i != scan->nb_points; i++) {
        //convert local lidar to global cordinates orientation
        x2p = c * scan->x[i] - s * scan->y[i];
        y2p = s * scan->x[i] + c * scan->y[i];
        //convert global lidar vlaues to grid cells
        xp = (int)floor((pos->x + x2p) * TS_MAP_SCALE + 0.5);
        yp = (int)floor((pos->y + y2p) * TS_MAP_SCALE + 0.5);
        //lidar distance of beam
        dist = sqrt(x2p * x2p + y2p * y2p);
        //the relative extension you add to make xp,yp extend past obstacle
        add = hole_width / 2 / dist;
        x2p *= TS_MAP_SCALE * (1 + add); //*= is the same as += but for muiltplication
        y2p *= TS_MAP_SCALE * (1 + add); 

        x2 = (int)floor(pos->x * TS_MAP_SCALE + x2p + 0.5); //extended x2, y2 values
        y2 = (int)floor(pos->y * TS_MAP_SCALE + y2p + 0.5);
        if (scan->value[i] == TS_NO_OBSTACLE) { 
            q = quality / 4;
            value = TS_NO_OBSTACLE;
        } else {
            q = quality;
            value = TS_OBSTACLE;
        }
        //printf("%d %d %d %d %d %d %d\n", i, x1, y1, x2, y2, xp, yp);
        ts_map_laser_ray(map, x1, y1, x2, y2, xp, yp, value, q);
    }
}

