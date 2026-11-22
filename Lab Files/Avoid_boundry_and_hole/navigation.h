/*
 * navigation.h
 *
 * Encoder-primary + dual-sensor (IR+PING) warehouse navigation system.
 * Assumes bot starts in the Bottom-Left (SW) corner at (22.5, 22.5) facing North (0 deg).
 *
 * Coordinate Convention:
 *   Origin (0, 0) = Bottom-Left (South-West) corner
 *   +X = East  (left -> right)
 *   +Y = North (bottom -> top)
 *   Arena Bearing: 0 deg = North (+Y), 90 deg = East (+X),
 *                  180 deg = South (-Y), 270 deg = West (-X)
 */

#ifndef NAVIGATION_H_
#define NAVIGATION_H_

#include <stdint.h>
#include "open_interface.h"

// =========================================================================
// Arena & Robot Geometry (cm)
// =========================================================================
// Bottom racks center Y = 31 (height 62 -> 0..62)
// Top racks center Y = 231   (height 62 -> 200..262) => ARENA_HEIGHT_CM = 262
// TODO: Measure exact total arena X width if different from 525 cm.
#define ARENA_WIDTH_CM       525.0f
#define ARENA_HEIGHT_CM      262.0f

// Default starting pose: Bottom-Left (SW) corner facing North (+Y)
#define START_X_CM           22.5f
#define START_Y_CM           22.5f
#define START_BEARING_DEG    0.0f

#define ROBOT_RADIUS_CM      17.0f
#define RACK_WIDTH_X_CM      31.0f
#define RACK_HEIGHT_Y_CM     62.0f

// Clearance inflation around racks/obstacles for grid planning
#define OBSTACLE_INFLATE_CM  21.0f
// Center-to-rack-edge standoff before PING creep (28 - 17 = 11 cm bumper gap)
#define RACK_STANDOFF_CM     28.0f
// Final bumper-to-rack gap after PING creep ("within a few cm")
#define RACK_FINAL_GAP_CM    6.0f

// Occupancy grid dimensions (15 cm cells -> 35 x 18 = 630 cells)
#define CELL_SIZE_CM         15.0f
#define GRID_COLS            35
#define GRID_ROWS            18

// Cell state values
#define CELL_FREE            0
#define CELL_VISITED         1
#define CELL_WALL            2
#define CELL_RACK            3
#define CELL_OBSTACLE        4  // Transient scanned obstacle
#define CELL_BUMP            5  // Persistent bumped short object
#define CELL_HOLE            6  // Persistent black floor hole

// Scan arrays shared between main.c and navigation.c
#define SCAN_POINTS          91 // 0..180 deg in 2-deg steps
#define MAX_DETECTED_OBJECTS 16

typedef struct {
    int id;
    int start_angle;
    int end_angle;
    int center_angle;
    float distance;
    int radial_width;
    float linear_width;
} ScanObject;

typedef enum {
    RACK_BL = 0, // Bottom-Left:  center (182, 31)
    RACK_TL = 1, // Top-Left:     center (182, 231)
    RACK_BR = 2, // Bottom-Right: center (343, 31)
    RACK_TR = 3  // Top-Right:    center (343, 231)
} RackId;

typedef enum {
    SIDE_NORTH = 0,
    SIDE_EAST  = 1, // Right side of rack
    SIDE_SOUTH = 2,
    SIDE_WEST  = 3  // Left side of rack
} RackSide;

typedef struct {
    float x;
    float y;
    float bearing_deg; // 0=N, 90=E, 180=S, 270=W
} BotPose;

// Scan globals populated by scan_field() in main.c
extern ScanObject detected_objects[MAX_DETECTED_OBJECTS];
extern int object_count;
extern float scan_ir_cm[SCAN_POINTS];
extern float scan_ping_cm[SCAN_POINTS];
extern float scan_fused_cm[SCAN_POINTS];
void scan_field(int verbose_raw);

// Initialization & pose helpers
void    nav_init_sw_corner(void);
void    nav_set_pose(float x_cm, float y_cm, float bearing_deg);
BotPose nav_get_pose(void);
void    nav_send_telemetry(void);

// Destination tracking
int  nav_is_dest_found(void);
void nav_get_dest(float *out_x, float *out_y);

// Rack target calculation (Left = SIDE_WEST, Right = SIDE_EAST)
int nav_get_rack_target(RackId rack, RackSide side,
                        float *out_x, float *out_y,
                        float *out_approach_bearing);

// Corridor safety helper (checks latest scan directly ahead of robot)
float nav_check_front_corridor_cm(float max_check_cm, int *out_hit_angle);

// Core navigation routines
int nav_drive_to_coord(oi_t *sensor, float goal_x, float goal_y,
                       int do_scans, int stop_on_dest);
int nav_approach_rack(oi_t *sensor, RackId rack, RackSide side);
int nav_find_destination(oi_t *sensor);
int nav_run_mission(oi_t *sensor,
                    RackId rack1, RackSide side1,
                    RackId rack2, RackSide side2);

#endif /* NAVIGATION_H_ */
