/*
 * navigation.h
 *
 * Arena map, occupancy grid + BFS path planner, IMU+encoder odometry,
 * two-wall localization, rack approach, and destination sweeping.
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
// Bottom racks center Y = 31 (height 62 -> 0..62),
// Top racks center Y = 231 (height 62 -> 200..262) => ARENA_HEIGHT_CM = 262.
// TODO: Verify ARENA_WIDTH_CM with a tape measure (525 assumes symmetric
//       left/right margins around x=182 and x=343: 182 + 343 = 525).
#define ARENA_WIDTH_CM       525.0f
#define ARENA_HEIGHT_CM      262.0f

#define ROBOT_RADIUS_CM      17.0f
#define RACK_WIDTH_X_CM      31.0f
#define RACK_HEIGHT_Y_CM     62.0f

// Clearance inflation around racks/obstacles for grid planning
#define OBSTACLE_INFLATE_CM  21.0f
// Center-to-rack-edge standoff before final creep (28 - 17 = 11 cm bumper gap)
#define RACK_STANDOFF_CM     28.0f
// Final bumper-to-rack distance after PING creep ("within a few cm")
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
#define CELL_OBSTACLE        4
#define CELL_HOLE            5

// Maximum scanned objects per 180-degree sweep
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
    SIDE_EAST  = 1,
    SIDE_SOUTH = 2,
    SIDE_WEST  = 3
} RackSide;

typedef struct {
    float x;
    float y;
    float bearing_deg; // 0=N, 90=E, 180=S, 270=W
} BotPose;

// Scan function implemented in main.c and called by navigation.c
extern ScanObject detected_objects[MAX_DETECTED_OBJECTS];
extern int object_count;
void scan_field(void);

// Initialization & pose helpers
void nav_init(float south_imu_deg);
void nav_set_pose(float x_cm, float y_cm);
BotPose nav_get_pose(void);
float nav_bearing_to_imu(float arena_bearing_deg);
float nav_imu_to_bearing(float imu_deg);
void nav_send_telemetry(void);

// Destination tracking (passive detection during all moves + active sweep)
int  nav_is_dest_found(void);
void nav_get_dest(float *out_x, float *out_y);

// Rack target calculation
int nav_get_rack_target(RackId rack, RackSide side,
                        float *out_x, float *out_y,
                        float *out_approach_bearing);

// Core navigation routines
int nav_drive_to_coord(oi_t *sensor, float goal_x, float goal_y,
                       int do_scans, int stop_on_dest);
int nav_localize_two_walls(oi_t *sensor);
int nav_approach_rack(oi_t *sensor, RackId rack, RackSide side);
int nav_find_destination(oi_t *sensor);
int nav_run_mission(oi_t *sensor,
                    RackId rack1, RackSide side1,
                    RackId rack2, RackSide side2);

#endif /* NAVIGATION_H_ */
