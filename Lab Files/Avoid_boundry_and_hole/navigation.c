/*
 * navigation.c
 *
 * Grid-based BFS path planner + IMU/encoder odometry + obstacle/hole avoidance
 * for the Autonomous Warehouse Bot (CPRE 288).
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "Timer.h"
#include "lcd.h"
#include "uart.h"
#include "servo.h"
#include "ping.h"
#include "BNO055.h"
#include "movement.h"
#include "navigation.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG_TO_RAD(d) ((d) * ((float)M_PI / 180.0f))
#define RAD_TO_DEG(r) ((r) * (180.0f / (float)M_PI))

#define DRIVE_SPEED          150
#define CREEP_SPEED          80
#define MAX_BURST_CM         50.0f
#define GOAL_TOL_CM          10.0f
#define MAX_SCAN_MARK_CM     115.0f
#define MAX_PATH_LEN         160

typedef struct {
    float cx;
    float cy;
    float half_x;
    float half_y;
} RackBox;

// Exact rack centers and half-extents (31 cm x 62 cm)
static const RackBox RACKS[4] = {
    { 182.0f,  31.0f, RACK_WIDTH_X_CM * 0.5f, RACK_HEIGHT_Y_CM * 0.5f }, // RACK_BL
    { 182.0f, 231.0f, RACK_WIDTH_X_CM * 0.5f, RACK_HEIGHT_Y_CM * 0.5f }, // RACK_TL
    { 343.0f,  31.0f, RACK_WIDTH_X_CM * 0.5f, RACK_HEIGHT_Y_CM * 0.5f }, // RACK_BR
    { 343.0f, 231.0f, RACK_WIDTH_X_CM * 0.5f, RACK_HEIGHT_Y_CM * 0.5f }  // RACK_TR
};

// Navigation state (static globals to keep TM4C stack usage tiny)
static uint8_t grid[GRID_ROWS][GRID_COLS];
static int16_t bfs_queue[GRID_ROWS * GRID_COLS];
static int16_t bfs_parent[GRID_ROWS * GRID_COLS];
static int16_t raw_path[MAX_PATH_LEN];
static int16_t smooth_path[MAX_PATH_LEN];

static BotPose bot_pose = { 25.0f, 25.0f, 0.0f };
static float north_imu_deg = 0.0f;

static int   dest_found = 0;
static float dest_x_cm  = 0.0f;
static float dest_y_cm  = 0.0f;

// =========================================================================
// Coordinate & Grid Helpers
// =========================================================================

float nav_bearing_to_imu(float arena_bearing_deg)
{
    return normalize_deg(north_imu_deg + arena_bearing_deg);
}

float nav_imu_to_bearing(float imu_deg)
{
    return normalize_deg(imu_deg - north_imu_deg);
}

static int cm_to_col(float x)
{
    int c = (int)(x / CELL_SIZE_CM);
    if (c < 0) c = 0;
    if (c >= GRID_COLS) c = GRID_COLS - 1;
    return c;
}

static int cm_to_row(float y)
{
    int r = (int)(y / CELL_SIZE_CM);
    if (r < 0) r = 0;
    if (r >= GRID_ROWS) r = GRID_ROWS - 1;
    return r;
}

static float col_to_cm(int c)
{
    return ((float)c + 0.5f) * CELL_SIZE_CM;
}

static float row_to_cm(int r)
{
    return ((float)r + 0.5f) * CELL_SIZE_CM;
}

static int cell_is_walkable(int r, int c)
{
    if (r < 0 || r >= GRID_ROWS || c < 0 || c >= GRID_COLS) return 0;
    return (grid[r][c] == CELL_FREE || grid[r][c] == CELL_VISITED);
}

static int point_inside_any_rack(float x, float y, float margin)
{
    int i;
    for (i = 0; i < 4; i++)
    {
        if (x >= (RACKS[i].cx - RACKS[i].half_x - margin) &&
            x <= (RACKS[i].cx + RACKS[i].half_x + margin) &&
            y >= (RACKS[i].cy - RACKS[i].half_y - margin) &&
            y <= (RACKS[i].cy + RACKS[i].half_y + margin))
        {
            return 1;
        }
    }
    return 0;
}

static void mark_circle(float cx, float cy, float radius_cm, uint8_t cell_type)
{
    int c_min = cm_to_col(cx - radius_cm);
    int c_max = cm_to_col(cx + radius_cm);
    int r_min = cm_to_row(cy - radius_cm);
    int r_max = cm_to_row(cy + radius_cm);
    int r, c;

    for (r = r_min; r <= r_max; r++)
    {
        for (c = c_min; c <= c_max; c++)
        {
            // Never overwrite permanent walls or racks
            if (grid[r][c] == CELL_WALL || grid[r][c] == CELL_RACK)
            {
                continue;
            }
            float dx = col_to_cm(c) - cx;
            float dy = row_to_cm(r) - cy;
            if ((dx * dx + dy * dy) <= (radius_cm * radius_cm))
            {
                grid[r][c] = cell_type;
            }
        }
    }
}

static void clear_temp_obstacles(void)
{
    int r, c;
    for (r = 0; r < GRID_ROWS; r++)
    {
        for (c = 0; c < GRID_COLS; c++)
        {
            if (grid[r][c] == CELL_OBSTACLE)
            {
                grid[r][c] = CELL_FREE;
            }
        }
    }
}

// =========================================================================
// Odometry Callback (called every ~25 ms during moves)
// =========================================================================

static void odometry_step_cb(float delta_cm, float imu_heading_deg)
{
    float bearing = nav_imu_to_bearing(imu_heading_deg);
    float rad = DEG_TO_RAD(bearing);

    // Bearing 0 = +Y (North), 90 = +X (East)
    bot_pose.x += delta_cm * sinf(rad);
    bot_pose.y += delta_cm * cosf(rad);
    bot_pose.bearing_deg = bearing;

    // Clamp softly inside arena bounds
    if (bot_pose.x < 0.0f) bot_pose.x = 0.0f;
    if (bot_pose.x > ARENA_WIDTH_CM) bot_pose.x = ARENA_WIDTH_CM;
    if (bot_pose.y < 0.0f) bot_pose.y = 0.0f;
    if (bot_pose.y > ARENA_HEIGHT_CM) bot_pose.y = ARENA_HEIGHT_CM;

    // Mark traversed cell as visited
    int r = cm_to_row(bot_pose.y);
    int c = cm_to_col(bot_pose.x);
    if (grid[r][c] == CELL_FREE)
    {
        grid[r][c] = CELL_VISITED;
    }
}

// =========================================================================
// Public Init / Pose / Telemetry
// =========================================================================

void nav_init(float south_imu_deg)
{
    int r, c;

    // South is 180 deg clockwise from North
    north_imu_deg = normalize_deg(south_imu_deg + 180.0f);
    dest_found = 0;
    dest_x_cm = 0.0f;
    dest_y_cm = 0.0f;

    for (r = 0; r < GRID_ROWS; r++)
    {
        float y = row_to_cm(r);
        for (c = 0; c < GRID_COLS; c++)
        {
            float x = col_to_cm(c);

            // Outer arena boundary walls (inflated by robot radius + 2 cm)
            if (x < (ROBOT_RADIUS_CM + 2.0f) ||
                x > (ARENA_WIDTH_CM - ROBOT_RADIUS_CM - 2.0f) ||
                y < (ROBOT_RADIUS_CM + 2.0f) ||
                y > (ARENA_HEIGHT_CM - ROBOT_RADIUS_CM - 2.0f))
            {
                grid[r][c] = CELL_WALL;
            }
            else if (point_inside_any_rack(x, y, OBSTACLE_INFLATE_CM))
            {
                grid[r][c] = CELL_RACK;
            }
            else
            {
                grid[r][c] = CELL_FREE;
            }
        }
    }
}

void nav_set_pose(float x_cm, float y_cm)
{
    float heading = 0.0f;
    bno055_convert_float_euler_h_deg(&heading);
    bot_pose.x = x_cm;
    bot_pose.y = y_cm;
    bot_pose.bearing_deg = nav_imu_to_bearing(heading);
}

BotPose nav_get_pose(void)
{
    float heading = 0.0f;
    bno055_convert_float_euler_h_deg(&heading);
    bot_pose.bearing_deg = nav_imu_to_bearing(heading);
    return bot_pose;
}

int nav_is_dest_found(void)
{
    return dest_found;
}

void nav_get_dest(float *out_x, float *out_y)
{
    if (out_x) *out_x = dest_x_cm;
    if (out_y) *out_y = dest_y_cm;
}

void nav_send_telemetry(void)
{
    char buf[80];
    BotPose p = nav_get_pose();
    sprintf(buf, "POS %.1f %.1f %.1f\r\n", p.x, p.y, p.bearing_deg);
    uart_sendStr(buf);
}

// =========================================================================
// Rack Side Targets
// =========================================================================

int nav_get_rack_target(RackId rack, RackSide side,
                        float *out_x, float *out_y,
                        float *out_approach_bearing)
{
    if (rack < RACK_BL || rack > RACK_TR) return 0;

    // South side of bottom racks and North side of top racks touch the wall
    if ((rack == RACK_BL || rack == RACK_BR) && side == SIDE_SOUTH) return 0;
    if ((rack == RACK_TL || rack == RACK_TR) && side == SIDE_NORTH) return 0;

    const RackBox *rb = &RACKS[rack];
    float tx = rb->cx;
    float ty = rb->cy;
    float bearing = 0.0f;

    switch (side)
    {
        case SIDE_WEST:
            tx = rb->cx - rb->half_x - RACK_STANDOFF_CM;
            ty = rb->cy;
            bearing = 90.0f;  // Face East toward rack
            break;
        case SIDE_EAST:
            tx = rb->cx + rb->half_x + RACK_STANDOFF_CM;
            ty = rb->cy;
            bearing = 270.0f; // Face West toward rack
            break;
        case SIDE_NORTH:
            tx = rb->cx;
            ty = rb->cy + rb->half_y + RACK_STANDOFF_CM;
            bearing = 180.0f; // Face South toward rack
            break;
        case SIDE_SOUTH:
            tx = rb->cx;
            ty = rb->cy - rb->half_y - RACK_STANDOFF_CM;
            bearing = 0.0f;   // Face North toward rack
            break;
        default:
            return 0;
    }

    if (out_x) *out_x = tx;
    if (out_y) *out_y = ty;
    if (out_approach_bearing) *out_approach_bearing = bearing;
    return 1;
}

// =========================================================================
// Sensor-to-Map Projection
// =========================================================================

static void mark_scanned_objects(float protect_goal_x, float protect_goal_y)
{
    BotPose p = nav_get_pose();
    char buf[80];
    int i;

    for (i = 0; i < object_count; i++)
    {
        float dist = detected_objects[i].distance;
        if (dist <= 5.0f || dist > MAX_SCAN_MARK_CM)
        {
            continue;
        }

        // Servo 0 = right (+90 CW), 90 = front (0), 180 = left (-90 CCW)
        float ray_bearing = normalize_deg(p.bearing_deg + (90.0f - (float)detected_objects[i].center_angle));
        float rad = DEG_TO_RAD(ray_bearing);

        // Sensor sits near front edge (~10 cm ahead of robot center)
        float total_r = dist + 10.0f;
        float ox = p.x + total_r * sinf(rad);
        float oy = p.y + total_r * cosf(rad);

        // Skip readings outside the arena or already belonging to a known rack
        if (ox < 8.0f || ox > (ARENA_WIDTH_CM - 8.0f) ||
            oy < 8.0f || oy > (ARENA_HEIGHT_CM - 8.0f))
        {
            continue;
        }
        if (point_inside_any_rack(ox, oy, 10.0f))
        {
            continue;
        }

        // Don't accidentally block our immediate standoff goal cell
        float gdx = ox - protect_goal_x;
        float gdy = oy - protect_goal_y;
        if ((gdx * gdx + gdy * gdy) < (18.0f * 18.0f))
        {
            continue;
        }

        float obj_rad = (detected_objects[i].linear_width * 0.5f) + ROBOT_RADIUS_CM;
        if (obj_rad < 18.0f) obj_rad = 18.0f;
        if (obj_rad > 32.0f) obj_rad = 32.0f;

        mark_circle(ox, oy, obj_rad, CELL_OBSTACLE);

        sprintf(buf, "OBS %.1f %.1f %.1f\r\n", ox, oy, detected_objects[i].linear_width);
        uart_sendStr(buf);
    }
}

static void mark_hazard_from_flag(int move_flag)
{
    BotPose p = nav_get_pose();
    float offset_deg = 0.0f;
    uint8_t cell_type = CELL_OBSTACLE;
    float mark_r = 20.0f;

    switch (move_flag)
    {
        case MOVE_FLAG_BUMP_LEFT:
            offset_deg = -30.0f;
            cell_type = CELL_OBSTACLE;
            break;
        case MOVE_FLAG_BUMP_RIGHT:
            offset_deg = 30.0f;
            cell_type = CELL_OBSTACLE;
            break;
        case MOVE_FLAG_BUMP_BOTH:
            offset_deg = 0.0f;
            cell_type = CELL_OBSTACLE;
            break;
        case MOVE_FLAG_HOLE_LEFT:
            offset_deg = -40.0f;
            cell_type = CELL_HOLE;
            mark_r = 24.0f;
            break;
        case MOVE_FLAG_HOLE_RIGHT:
            offset_deg = 40.0f;
            cell_type = CELL_HOLE;
            mark_r = 24.0f;
            break;
        case MOVE_FLAG_HOLE_FRONT:
            offset_deg = 0.0f;
            cell_type = CELL_HOLE;
            mark_r = 24.0f;
            break;
        default:
            return; // Walls are already permanent on the map
    }

    float rad = DEG_TO_RAD(normalize_deg(p.bearing_deg + offset_deg));
    float hx = p.x + (ROBOT_RADIUS_CM + 6.0f) * sinf(rad);
    float hy = p.y + (ROBOT_RADIUS_CM + 6.0f) * cosf(rad);

    mark_circle(hx, hy, mark_r, cell_type);

    char buf[64];
    sprintf(buf, "HAZ %d %.1f %.1f\r\n", move_flag, hx, hy);
    uart_sendStr(buf);
}

// =========================================================================
// BFS Path Planner + Line-of-Sight Waypoint Compression
// =========================================================================

static int find_nearest_walkable(int r0, int c0, int *out_r, int *out_c)
{
    if (cell_is_walkable(r0, c0))
    {
        *out_r = r0;
        *out_c = c0;
        return 1;
    }

    int radius, dr, dc;
    for (radius = 1; radius <= 4; radius++)
    {
        for (dr = -radius; dr <= radius; dr++)
        {
            for (dc = -radius; dc <= radius; dc++)
            {
                if (cell_is_walkable(r0 + dr, c0 + dc))
                {
                    *out_r = r0 + dr;
                    *out_c = c0 + dc;
                    return 1;
                }
            }
        }
    }
    return 0;
}

static int line_of_sight_clear(int r0, int c0, int r1, int c1)
{
    float x0 = col_to_cm(c0), y0 = row_to_cm(r0);
    float x1 = col_to_cm(c1), y1 = row_to_cm(r1);
    float dx = x1 - x0;
    float dy = y1 - y0;
    float dist = sqrtf(dx * dx + dy * dy);
    int steps = (int)(dist / 5.0f) + 1;
    int s;

    for (s = 1; s < steps; s++)
    {
        float t = (float)s / (float)steps;
        int rc = cm_to_row(y0 + t * dy);
        int cc = cm_to_col(x0 + t * dx);
        if (!cell_is_walkable(rc, cc))
        {
            return 0;
        }
    }
    return 1;
}

// Plans a path from current bot cell to (goal_x, goal_y).
// Populates smooth_path[] with compressed cell indices (start..goal).
// Returns number of waypoints in smooth_path[], or 0 if unreachable.
static int plan_bfs_path(float goal_x, float goal_y)
{
    const int total_cells = GRID_ROWS * GRID_COLS;
    const int dr[4] = { 1, 0, -1, 0 }; // N, E, S, W
    const int dc[4] = { 0, 1, 0, -1 };

    int sr = cm_to_row(bot_pose.y);
    int sc = cm_to_col(bot_pose.x);
    int gr = cm_to_row(goal_y);
    int gc = cm_to_col(goal_x);

    if (!find_nearest_walkable(gr, gc, &gr, &gc))
    {
        return 0;
    }

    int start_idx = sr * GRID_COLS + sc;
    int goal_idx  = gr * GRID_COLS + gc;
    if (start_idx == goal_idx)
    {
        smooth_path[0] = (int16_t)goal_idx;
        return 1;
    }

    int i;
    for (i = 0; i < total_cells; i++)
    {
        bfs_parent[i] = -1;
    }

    int head = 0, tail = 0;
    bfs_queue[tail++] = (int16_t)start_idx;
    bfs_parent[start_idx] = (int16_t)start_idx;

    int found = 0;
    while (head < tail)
    {
        int curr = bfs_queue[head++];
        if (curr == goal_idx)
        {
            found = 1;
            break;
        }

        int cr = curr / GRID_COLS;
        int cc = curr % GRID_COLS;
        int d;
        for (d = 0; d < 4; d++)
        {
            int nr = cr + dr[d];
            int nc = cc + dc[d];
            if (nr < 0 || nr >= GRID_ROWS || nc < 0 || nc >= GRID_COLS)
            {
                continue;
            }
            int nidx = nr * GRID_COLS + nc;
            if (bfs_parent[nidx] != -1)
            {
                continue;
            }
            if (!cell_is_walkable(nr, nc) && nidx != goal_idx)
            {
                continue;
            }
            bfs_parent[nidx] = (int16_t)curr;
            bfs_queue[tail++] = (int16_t)nidx;
        }
    }

    if (!found)
    {
        return 0;
    }

    // Reconstruct backwards from goal to start
    int raw_len = 0;
    int cur = goal_idx;
    while (cur != start_idx && raw_len < MAX_PATH_LEN - 1)
    {
        raw_path[raw_len++] = (int16_t)cur;
        cur = bfs_parent[cur];
    }
    raw_path[raw_len++] = (int16_t)start_idx;

    // Reverse into start -> goal order
    for (i = 0; i < raw_len / 2; i++)
    {
        int16_t tmp = raw_path[i];
        raw_path[i] = raw_path[raw_len - 1 - i];
        raw_path[raw_len - 1 - i] = tmp;
    }

    // Greedy line-of-sight smoothing to produce clean straight segments
    int smooth_len = 0;
    int anchor = 0;
    smooth_path[smooth_len++] = raw_path[0];

    while (anchor < raw_len - 1 && smooth_len < MAX_PATH_LEN)
    {
        int ar = raw_path[anchor] / GRID_COLS;
        int ac = raw_path[anchor] % GRID_COLS;
        int next = raw_len - 1;

        while (next > anchor + 1)
        {
            int nr = raw_path[next] / GRID_COLS;
            int nc = raw_path[next] % GRID_COLS;
            if (line_of_sight_clear(ar, ac, nr, nc))
            {
                break;
            }
            next--;
        }

        smooth_path[smooth_len++] = raw_path[next];
        anchor = next;
    }

    return smooth_len;
}

// =========================================================================
// Core Drive-To-Coordinate Loop (Milestone 1!)
// =========================================================================

// Returns: 1 = reached goal, 2 = stopped on destination mat, 0 = failed
int nav_drive_to_coord(oi_t *sensor, float goal_x, float goal_y,
                       int do_scans, int stop_on_dest)
{
    int iter = 0;
    int cleared_once = 0;
    char buf[80];

    sprintf(buf, "GOAL %.1f %.1f\r\n", goal_x, goal_y);
    uart_sendStr(buf);

    while (iter < 80)
    {
        iter++;
        float dx_goal = goal_x - bot_pose.x;
        float dy_goal = goal_y - bot_pose.y;
        float dist_goal = sqrtf(dx_goal * dx_goal + dy_goal * dy_goal);

        nav_send_telemetry();
        lcd_printf("Pos:%.0f,%.0f\nGoal:%.0f,%.0f\nDist:%.0fcm",
                   bot_pose.x, bot_pose.y, goal_x, goal_y, dist_goal);

        if (dist_goal <= GOAL_TOL_CM)
        {
            uart_sendStr("ARRIVED\r\n");
            return 1;
        }

        // Plan path through occupancy grid
        int path_len = plan_bfs_path(goal_x, goal_y);
        if (path_len == 0)
        {
            if (!cleared_once)
            {
                // Clear transient scan obstacles (keep racks/walls/holes) and retry
                uart_sendStr("REPLAN_CLEAR\r\n");
                clear_temp_obstacles();
                cleared_once = 1;
                continue;
            }
            uart_sendStr("NO_PATH\r\n");
            return 0;
        }

        // Pick next waypoint (index 1, or exact goal if at final segment)
        float wp_x = goal_x;
        float wp_y = goal_y;
        if (path_len >= 2)
        {
            int wp_idx = smooth_path[1];
            int wp_r = wp_idx / GRID_COLS;
            int wp_c = wp_idx % GRID_COLS;
            if (path_len > 2)
            {
                wp_x = col_to_cm(wp_c);
                wp_y = row_to_cm(wp_r);
            }
        }

        float dx = wp_x - bot_pose.x;
        float dy = wp_y - bot_pose.y;
        float seg_dist = sqrtf(dx * dx + dy * dy);
        if (seg_dist < 3.0f)
        {
            continue;
        }

        // Turn toward waypoint using BNO055 IMU
        float target_bearing = normalize_deg(RAD_TO_DEG(atan2f(dx, dy)));
        float target_imu = nav_bearing_to_imu(target_bearing);
        turn_to_heading(sensor, target_imu);

        // Scan forward hemisphere before driving a burst
        if (do_scans)
        {
            scan_field();
            mark_scanned_objects(goal_x, goal_y);

            // Re-check if scan just revealed an obstacle directly on our segment
            int new_len = plan_bfs_path(goal_x, goal_y);
            if (new_len >= 2 && new_len != path_len)
            {
                int new_idx = smooth_path[1];
                float ndx = col_to_cm(new_idx % GRID_COLS) - bot_pose.x;
                float ndy = row_to_cm(new_idx / GRID_COLS) - bot_pose.y;
                float new_bearing = normalize_deg(RAD_TO_DEG(atan2f(ndx, ndy)));
                if (fabsf(angle_diff_deg(new_bearing, target_bearing)) > 20.0f)
                {
                    // Obstacle directly ahead -> replan immediately before moving
                    continue;
                }
            }
        }

        // Drive capped burst length so we re-scan and stay accurate
        float burst = (seg_dist > MAX_BURST_CM) ? MAX_BURST_CM : seg_dist;
        move_forward_imu(sensor, burst, DRIVE_SPEED, target_imu, odometry_step_cb);

        // Handle stop condition
        if (flag == MOVE_FLAG_DESTINATION)
        {
            dest_found = 1;
            dest_x_cm = bot_pose.x;
            dest_y_cm = bot_pose.y;
            sprintf(buf, "DEST_FOUND %.1f %.1f\r\n", dest_x_cm, dest_y_cm);
            uart_sendStr(buf);

            if (stop_on_dest)
            {
                return 2;
            }
        }
        else if (flag != MOVE_FLAG_NONE)
        {
            // Bump, hole, or unexpected wall: mark hazard, back up, and replan
            mark_hazard_from_flag(flag);
            move_backward_odometry(sensor, 12.0, DRIVE_SPEED, odometry_step_cb);
            cleared_once = 0;
        }
    }

    return 0;
}

// =========================================================================
// Two-Wall Localization (works from any starting position & orientation!)
// =========================================================================

// Helper: measure average front clearance (70..110 deg) using PING
static float measure_front_clearance_cm(void)
{
    float min_cm = 250.0f;
    int deg;
    for (deg = 70; deg <= 110; deg += 10)
    {
        servo_move((float)deg);
        timer_waitMillis(60);
        float d = ping_read_cm();
        if (d > 0.0f && d < min_cm)
        {
            min_cm = d;
        }
    }
    servo_move(90.0f);
    return min_cm;
}

// Drives along a cardinal bearing until white wall tape is hit,
// dodging bumps/holes sideways while tracking odometry.
// Returns 1 if wall reached, 0 on failure.
static int drive_to_wall(oi_t *sensor, float wall_bearing_deg, int track_odom)
{
    float wall_imu = nav_bearing_to_imu(wall_bearing_deg);
    int attempts = 0;

    while (attempts < 12)
    {
        attempts++;
        turn_to_heading(sensor, wall_imu);

        move_forward_imu(sensor, 400.0, DRIVE_SPEED, wall_imu,
                         track_odom ? odometry_step_cb : 0);

        if (flag == MOVE_FLAG_WALL_FRONT ||
            flag == MOVE_FLAG_WALL_LEFT  ||
            flag == MOVE_FLAG_WALL_RIGHT)
        {
            return 1;
        }

        if (flag == MOVE_FLAG_DESTINATION)
        {
            // Keep going; note we can't store world dest until localized
            move_forward_imu(sensor, 20.0, DRIVE_SPEED, wall_imu,
                             track_odom ? odometry_step_cb : 0);
            continue;
        }

        if (flag != MOVE_FLAG_NONE)
        {
            // Obstacle or hole blocking path to wall -> back up and sidestep 35 cm
            move_backward_odometry(sensor, 14.0, DRIVE_SPEED,
                                   track_odom ? odometry_step_cb : 0);

            float dodge_bearing = normalize_deg(wall_bearing_deg + 90.0f);
            float dodge_imu = nav_bearing_to_imu(dodge_bearing);
            turn_to_heading(sensor, dodge_imu);

            if (measure_front_clearance_cm() < 40.0f)
            {
                dodge_bearing = normalize_deg(wall_bearing_deg - 90.0f);
                dodge_imu = nav_bearing_to_imu(dodge_bearing);
                turn_to_heading(sensor, dodge_imu);
            }

            move_forward_imu(sensor, 35.0, DRIVE_SPEED, dodge_imu,
                             track_odom ? odometry_step_cb : 0);
        }
    }
    return 0;
}

int nav_localize_two_walls(oi_t *sensor)
{
    uart_sendStr("LOCALIZING_START\r\n");
    lcd_printf("Localizing...\nStep 1: X wall");

    // Step 1: Compare West (270) vs East (90) clearance.
    // Prefer West because West wall is always x = ROBOT_RADIUS_CM regardless of total width.
    turn_to_heading(sensor, nav_bearing_to_imu(270.0f));
    float west_clear = measure_front_clearance_cm();

    float x_wall_bearing = 270.0f;
    if (west_clear < 35.0f)
    {
        turn_to_heading(sensor, nav_bearing_to_imu(90.0f));
        float east_clear = measure_front_clearance_cm();
        if (east_clear > west_clear)
        {
            x_wall_bearing = 90.0f;
        }
    }

    if (!drive_to_wall(sensor, x_wall_bearing, 0))
    {
        uart_sendStr("LOCALIZE_FAIL_X\r\n");
        return 0;
    }

    // Lock X coordinate from the wall we just touched
    if (x_wall_bearing == 270.0f)
    {
        bot_pose.x = ROBOT_RADIUS_CM;
    }
    else
    {
        bot_pose.x = ARENA_WIDTH_CM - ROBOT_RADIUS_CM;
    }
    bot_pose.y = ARENA_HEIGHT_CM * 0.5f; // Temporary until Step 2 locks Y

    // Back off wall cleanly (now tracking X odometry!)
    move_backward_odometry(sensor, 15.0, DRIVE_SPEED, odometry_step_cb);

    // Step 2: Compare South (180) vs North (0) clearance and drive to Y wall
    lcd_printf("Localizing...\nStep 2: Y wall");
    turn_to_heading(sensor, nav_bearing_to_imu(180.0f));
    float south_clear = measure_front_clearance_cm();

    float y_wall_bearing = 180.0f;
    if (south_clear < 35.0f)
    {
        turn_to_heading(sensor, nav_bearing_to_imu(0.0f));
        float north_clear = measure_front_clearance_cm();
        if (north_clear > south_clear)
        {
            y_wall_bearing = 0.0f;
        }
    }

    if (!drive_to_wall(sensor, y_wall_bearing, 1))
    {
        uart_sendStr("LOCALIZE_FAIL_Y\r\n");
        return 0;
    }

    // Lock Y coordinate from the wall we just touched
    if (y_wall_bearing == 180.0f)
    {
        bot_pose.y = ROBOT_RADIUS_CM;
    }
    else
    {
        bot_pose.y = ARENA_HEIGHT_CM - ROBOT_RADIUS_CM;
    }

    move_backward_odometry(sensor, 15.0, DRIVE_SPEED, odometry_step_cb);

    char buf[80];
    sprintf(buf, "LOCALIZED %.1f %.1f\r\n", bot_pose.x, bot_pose.y);
    uart_sendStr(buf);
    lcd_printf("Localized!\nX:%.1f Y:%.1f", bot_pose.x, bot_pose.y);
    return 1;
}

// =========================================================================
// Rack Approach + Free Coordinate Re-Calibration
// =========================================================================

int nav_approach_rack(oi_t *sensor, RackId rack, RackSide side)
{
    float tx = 0.0f, ty = 0.0f, approach_bearing = 0.0f;
    if (!nav_get_rack_target(rack, side, &tx, &ty, &approach_bearing))
    {
        uart_sendStr("INVALID_RACK_SIDE\r\n");
        return 0;
    }

    // 1. Navigate to standoff coordinate in front of the chosen rack side
    if (!nav_drive_to_coord(sensor, tx, ty, 1, 0))
    {
        return 0;
    }

    // 2. Turn to face the rack wall directly
    float approach_imu = nav_bearing_to_imu(approach_bearing);
    turn_to_heading(sensor, approach_imu);

    // 3. Measure PING straight ahead and creep to within ~6 cm of the rack
    servo_move(90.0f);
    timer_waitMillis(200);
    float d_cm = ping_read_cm();

    if (d_cm > RACK_FINAL_GAP_CM && d_cm < 45.0f)
    {
        float creep = d_cm - RACK_FINAL_GAP_CM;
        move_forward_imu(sensor, creep, CREEP_SPEED, approach_imu, odometry_step_cb);
        timer_waitMillis(100);
        d_cm = ping_read_cm();
    }

    // 4. Re-calibrate X or Y from known rack face + live PING distance!
    if (d_cm > 0.0f && d_cm < 30.0f)
    {
        const RackBox *rb = &RACKS[rack];
        float center_to_face = d_cm + ROBOT_RADIUS_CM;
        if (side == SIDE_WEST)       bot_pose.x = (rb->cx - rb->half_x) - center_to_face;
        else if (side == SIDE_EAST)  bot_pose.x = (rb->cx + rb->half_x) + center_to_face;
        else if (side == SIDE_NORTH) bot_pose.y = (rb->cy + rb->half_y) + center_to_face;
        else if (side == SIDE_SOUTH) bot_pose.y = (rb->cy - rb->half_y) - center_to_face;
    }

    char buf[80];
    sprintf(buf, "RACK_PICKUP %d %d POS %.1f %.1f\r\n",
            (int)rack, (int)side, bot_pose.x, bot_pose.y);
    uart_sendStr(buf);
    lcd_printf("Picked up!\nRack %d Side %d", (int)rack, (int)side);
    timer_waitMillis(1000);

    // 5. Back off safely to standoff before next leg
    move_backward_odometry(sensor, 14.0, DRIVE_SPEED, odometry_step_cb);
    return 1;
}

// =========================================================================
// Find Destination (Passive Hit Reuse or Unvisited-Cell Sweep)
// =========================================================================

int nav_find_destination(oi_t *sensor)
{
    // If we already drove over the destination earlier, go straight there!
    if (dest_found)
    {
        uart_sendStr("DEST_KNOWN_RETURNING\r\n");
        return nav_drive_to_coord(sensor, dest_x_cm, dest_y_cm, 1, 1) > 0;
    }

    uart_sendStr("SWEEPING_FOR_DEST\r\n");
    lcd_printf("Sweeping for\ndestination...");

    // Sweep open aisle waypoints (lawnmower order across unvisited areas)
    const float sweep_pts[][2] = {
        { 90.0f,  131.0f },
        { 90.0f,  210.0f },
        { 262.0f, 210.0f },
        { 262.0f, 131.0f },
        { 262.0f,  55.0f },
        { 430.0f,  55.0f },
        { 430.0f, 131.0f },
        { 430.0f, 210.0f },
        { 90.0f,   55.0f }
    };
    const int num_pts = (int)(sizeof(sweep_pts) / sizeof(sweep_pts[0]));
    int i;

    for (i = 0; i < num_pts; i++)
    {
        float sx = sweep_pts[i][0];
        float sy = sweep_pts[i][1];

        // Skip points outside current arena width or already visited
        if (sx > (ARENA_WIDTH_CM - 30.0f)) continue;
        if (grid[cm_to_row(sy)][cm_to_col(sx)] == CELL_VISITED) continue;

        int res = nav_drive_to_coord(sensor, sx, sy, 1, 1);
        if (res == 2 || dest_found)
        {
            return 1;
        }
    }

    uart_sendStr("DEST_NOT_FOUND\r\n");
    return 0;
}

// =========================================================================
// Full 2-Rack Mission
// =========================================================================

int nav_run_mission(oi_t *sensor,
                    RackId rack1, RackSide side1,
                    RackId rack2, RackSide side2)
{
    uart_sendStr("MISSION_START\r\n");

    // 1. Go to Rack 1 and pick up item 1
    if (!nav_approach_rack(sensor, rack1, side1))
    {
        uart_sendStr("MISSION_FAIL_RACK1\r\n");
        return 0;
    }

    // 2. Find destination mat and drop off item 1
    if (!nav_find_destination(sensor))
    {
        uart_sendStr("MISSION_FAIL_DEST1\r\n");
        return 0;
    }
    uart_sendStr("DROPOFF_1_COMPLETE\r\n");
    timer_waitMillis(1000);

    // 3. Go to Rack 2 and pick up item 2
    if (!nav_approach_rack(sensor, rack2, side2))
    {
        uart_sendStr("MISSION_FAIL_RACK2\r\n");
        return 0;
    }

    // 4. Return directly to saved destination coordinates
    if (!nav_drive_to_coord(sensor, dest_x_cm, dest_y_cm, 1, 1))
    {
        uart_sendStr("MISSION_FAIL_DEST2\r\n");
        return 0;
    }

    uart_sendStr("MISSION_COMPLETE\r\n");
    lcd_printf("ORDER COMPLETE!\nAt Packing Bay");

    // Audible completion alert on Roomba speaker
    unsigned char notes[4] = { 60, 64, 67, 72 };
    unsigned char dur[4]   = { 16, 16, 16, 32 };
    oi_loadSong(0, 4, notes, dur);
    oi_play_song(0);
    return 1;
}
