/*
 * navigation.c
 *
 * Encoder-primary + dual-sensor (IR+PING) grid BFS path planner with
 * active corridor collision checking and true sidestep obstacle detouring.
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "Timer.h"
#include "lcd.h"
#include "uart.h"
#include "servo.h"
#include "ping.h"
#include "movement.h"
#include "navigation.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG_TO_RAD(d) ((d) * ((float)M_PI / 180.0f))
#define RAD_TO_DEG(r) ((r) * (180.0f / (float)M_PI))

#define DRIVE_SPEED          140
#define CREEP_SPEED          75
#define MAX_BURST_CM         45.0f
#define GOAL_TOL_CM          12.0f
#define MAX_SCAN_MARK_CM     105.0f
#define CORRIDOR_HALF_W_CM   (ROBOT_RADIUS_CM + 4.0f)  // 21 cm half-width
#define FRONT_STOP_MARGIN_CM 15.0f
#define DETOUR_SIDE_CM       36.0f
#define DETOUR_PASS_CM       34.0f
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

// Static globals keep TM4C stack usage minimal
static uint8_t grid[GRID_ROWS][GRID_COLS];
static int16_t bfs_queue[GRID_ROWS * GRID_COLS];
static int16_t bfs_parent[GRID_ROWS * GRID_COLS];
static int16_t raw_path[MAX_PATH_LEN];
static int16_t smooth_path[MAX_PATH_LEN];

static BotPose bot_pose = { START_X_CM, START_Y_CM, START_BEARING_DEG };

static int   dest_found = 0;
static float dest_x_cm  = 0.0f;
static float dest_y_cm  = 0.0f;

// =========================================================================
// Grid & Geometry Helpers
// =========================================================================

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
            // Never overwrite permanent walls or racks; don't downgrade holes/bumps
            if (grid[r][c] == CELL_WALL || grid[r][c] == CELL_RACK)
            {
                continue;
            }
            if ((grid[r][c] == CELL_HOLE || grid[r][c] == CELL_BUMP) &&
                cell_type == CELL_OBSTACLE)
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

static void clear_temp_obstacles(int include_bumps)
{
    int r, c;
    for (r = 0; r < GRID_ROWS; r++)
    {
        for (c = 0; c < GRID_COLS; c++)
        {
            if (grid[r][c] == CELL_OBSTACLE ||
                (include_bumps && grid[r][c] == CELL_BUMP))
            {
                grid[r][c] = CELL_FREE;
            }
        }
    }
}

// =========================================================================
// Encoder-Primary Odometry Callback + Tracked Turn Wrapper
// =========================================================================

static void odometry_step_cb(float delta_cm, float delta_cw_deg)
{
    // Midpoint bearing integration for smooth curved motion tracking
    float mid_bearing = normalize_deg(bot_pose.bearing_deg + 0.5f * delta_cw_deg);
    float rad = DEG_TO_RAD(mid_bearing);

    // Bearing 0 = +Y (North), 90 = +X (East)
    bot_pose.x += delta_cm * sinf(rad);
    bot_pose.y += delta_cm * cosf(rad);
    bot_pose.bearing_deg = normalize_deg(bot_pose.bearing_deg + delta_cw_deg);

    if (bot_pose.x < 0.0f) bot_pose.x = 0.0f;
    if (bot_pose.x > ARENA_WIDTH_CM) bot_pose.x = ARENA_WIDTH_CM;
    if (bot_pose.y < 0.0f) bot_pose.y = 0.0f;
    if (bot_pose.y > ARENA_HEIGHT_CM) bot_pose.y = ARENA_HEIGHT_CM;

    int r = cm_to_row(bot_pose.y);
    int c = cm_to_col(bot_pose.x);
    if (grid[r][c] == CELL_FREE)
    {
        grid[r][c] = CELL_VISITED;
    }
}

static void nav_turn_to_bearing(oi_t *sensor, float target_bearing_deg)
{
    target_bearing_deg = normalize_deg(target_bearing_deg);
    float delta_cw = angle_diff_deg(target_bearing_deg, bot_pose.bearing_deg);

    if (fabsf(delta_cw) < 2.0f)
    {
        return;
    }

    float actual_cw = turn_by_degrees(sensor, delta_cw, 0, 0);
    bot_pose.bearing_deg = normalize_deg(bot_pose.bearing_deg + actual_cw);
}

// =========================================================================
// Public Init / Pose / Telemetry
// =========================================================================

void nav_init_sw_corner(void)
{
    int r, c;

    dest_found = 0;
    dest_x_cm = 0.0f;
    dest_y_cm = 0.0f;

    bot_pose.x = START_X_CM;
    bot_pose.y = START_Y_CM;
    bot_pose.bearing_deg = START_BEARING_DEG;

    for (r = 0; r < GRID_ROWS; r++)
    {
        float y = row_to_cm(r);
        for (c = 0; c < GRID_COLS; c++)
        {
            float x = col_to_cm(c);

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

void nav_set_pose(float x_cm, float y_cm, float bearing_deg)
{
    bot_pose.x = x_cm;
    bot_pose.y = y_cm;
    bot_pose.bearing_deg = normalize_deg(bearing_deg);
}

BotPose nav_get_pose(void)
{
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
    sprintf(buf, "POS %.1f %.1f %.1f\r\n", bot_pose.x, bot_pose.y, bot_pose.bearing_deg);
    uart_sendStr(buf);
}

// =========================================================================
// Rack Side Targets (Left = SIDE_WEST, Right = SIDE_EAST)
// =========================================================================

int nav_get_rack_target(RackId rack, RackSide side,
                        float *out_x, float *out_y,
                        float *out_approach_bearing)
{
    if (rack < RACK_BL || rack > RACK_TR) return 0;

    // South side of bottom racks and North side of top racks sit on the arena wall
    if ((rack == RACK_BL || rack == RACK_BR) && side == SIDE_SOUTH) return 0;
    if ((rack == RACK_TL || rack == RACK_TR) && side == SIDE_NORTH) return 0;

    const RackBox *rb = &RACKS[rack];
    float tx = rb->cx;
    float ty = rb->cy;
    float bearing = 0.0f;

    switch (side)
    {
        case SIDE_WEST: // Left side of rack
            tx = rb->cx - rb->half_x - RACK_STANDOFF_CM;
            ty = rb->cy;
            bearing = 90.0f;  // Face East toward rack
            break;
        case SIDE_EAST: // Right side of rack
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
// Scan-to-Map Projection & Direct Forward Corridor Clearance Check
// =========================================================================

// Inspects the raw fused scan array (PING primary + close IR) for UNKNOWN
// obstacles lying inside the robot's forward driving corridor.
// Known rack surfaces and outside-arena walls are skipped so the robot does not
// panic-detour away from the rack it is deliberately approaching or routing past.
float nav_check_front_corridor_cm(float max_check_cm, int *out_hit_angle)
{
    float safe_cm = max_check_cm;
    int hit_angle = -1;
    int idx;

    // Servo 0 = right, 90 = straight ahead, 180 = left
    // Check front cone 30 deg .. 150 deg (tighter than 20..160 to avoid wide PING side echoes)
    for (idx = 15; idx <= 75; idx++) // 30 deg .. 150 deg
    {
        float d = scan_fused_cm[idx];
        if (d <= 4.0f || d > (max_check_cm + FRONT_STOP_MARGIN_CM))
        {
            continue;
        }

        float deg = (float)(idx * 2);
        float rad = DEG_TO_RAD(deg);
        float fwd = d * sinf(rad);        // Ahead of sensor
        float lat = fabsf(d * cosf(rad)); // Sideways distance from centerline

        if (lat < CORRIDOR_HALF_W_CM)
        {
            // Project point into arena coordinates to check if it belongs to a known rack
            float ray_bearing = normalize_deg(bot_pose.bearing_deg + (90.0f - deg));
            float rrad = DEG_TO_RAD(ray_bearing);
            float ox = bot_pose.x + (d + 10.0f) * sinf(rrad);
            float oy = bot_pose.y + (d + 10.0f) * cosf(rrad);

            if (point_inside_any_rack(ox, oy, 14.0f))
            {
                continue; // Known rack (already planned around by BFS)
            }
            if (ox < 8.0f || ox > (ARENA_WIDTH_CM - 8.0f) ||
                oy < 8.0f || oy > (ARENA_HEIGHT_CM - 8.0f))
            {
                continue; // Outside arena wall
            }

            float allowable = fwd - FRONT_STOP_MARGIN_CM;
            if (allowable < 0.0f) allowable = 0.0f;
            if (allowable < safe_cm)
            {
                safe_cm = allowable;
                hit_angle = idx * 2;
            }
        }
    }

    if (out_hit_angle) *out_hit_angle = hit_angle;
    return safe_cm;
}

static void mark_scanned_objects(float protect_goal_x, float protect_goal_y)
{
    char buf[80];
    int i;

    for (i = 0; i < object_count; i++)
    {
        float dist = detected_objects[i].distance;
        if (dist <= 4.0f || dist > MAX_SCAN_MARK_CM)
        {
            continue;
        }

        float ray_bearing = normalize_deg(bot_pose.bearing_deg +
                                          (90.0f - (float)detected_objects[i].center_angle));
        float rad = DEG_TO_RAD(ray_bearing);

        // Sensor sits ~10 cm ahead of robot center
        float total_r = dist + 10.0f;
        float ox = bot_pose.x + total_r * sinf(rad);
        float oy = bot_pose.y + total_r * cosf(rad);

        if (ox < 8.0f || ox > (ARENA_WIDTH_CM - 8.0f) ||
            oy < 8.0f || oy > (ARENA_HEIGHT_CM - 8.0f))
        {
            continue;
        }
        if (point_inside_any_rack(ox, oy, 10.0f))
        {
            continue;
        }

        // Do not accidentally block our immediate standoff goal cell
        float gdx = ox - protect_goal_x;
        float gdy = oy - protect_goal_y;
        if ((gdx * gdx + gdy * gdy) < (16.0f * 16.0f))
        {
            continue;
        }

        // Generous inflation so BFS routes cleanly around pillars & boxes
        float obj_rad = (detected_objects[i].linear_width * 0.5f) + ROBOT_RADIUS_CM + 6.0f;
        if (obj_rad < 23.0f) obj_rad = 23.0f;
        if (obj_rad > 36.0f) obj_rad = 36.0f;

        mark_circle(ox, oy, obj_rad, CELL_OBSTACLE);

        sprintf(buf, "OBS %.1f %.1f d:%.1f w:%.1f\r\n",
                ox, oy, dist, detected_objects[i].linear_width);
        uart_sendStr(buf);
    }
}

static void mark_corridor_blocker(int hit_angle, float dist_cm)
{
    float ray_bearing = normalize_deg(bot_pose.bearing_deg + (90.0f - (float)hit_angle));
    float rad = DEG_TO_RAD(ray_bearing);
    float ox = bot_pose.x + (dist_cm + 10.0f) * sinf(rad);
    float oy = bot_pose.y + (dist_cm + 10.0f) * cosf(rad);

    if (!point_inside_any_rack(ox, oy, 8.0f))
    {
        mark_circle(ox, oy, 26.0f, CELL_OBSTACLE);
    }
}

static void mark_hazard_from_flag(int move_flag)
{
    float offset_deg = 0.0f;
    uint8_t cell_type = CELL_BUMP;
    float mark_r = 28.0f; // Large enough (~2 cells) so BFS truly routes around

    switch (move_flag)
    {
        case MOVE_FLAG_BUMP_LEFT:   offset_deg = -25.0f; cell_type = CELL_BUMP; break;
        case MOVE_FLAG_BUMP_RIGHT:  offset_deg =  25.0f; cell_type = CELL_BUMP; break;
        case MOVE_FLAG_BUMP_BOTH:   offset_deg =   0.0f; cell_type = CELL_BUMP; break;
        case MOVE_FLAG_HOLE_LEFT:   offset_deg = -35.0f; cell_type = CELL_HOLE; mark_r = 30.0f; break;
        case MOVE_FLAG_HOLE_RIGHT:  offset_deg =  35.0f; cell_type = CELL_HOLE; mark_r = 30.0f; break;
        case MOVE_FLAG_HOLE_FRONT:  offset_deg =   0.0f; cell_type = CELL_HOLE; mark_r = 30.0f; break;
        default: return;
    }

    float rad = DEG_TO_RAD(normalize_deg(bot_pose.bearing_deg + offset_deg));
    float hx = bot_pose.x + (ROBOT_RADIUS_CM + 8.0f) * sinf(rad);
    float hy = bot_pose.y + (ROBOT_RADIUS_CM + 8.0f) * cosf(rad);

    mark_circle(hx, hy, mark_r, cell_type);

    char buf[64];
    sprintf(buf, "HAZ %d %.1f %.1f\r\n", move_flag, hx, hy);
    uart_sendStr(buf);
}

// =========================================================================
// Active Detour Maneuver (Actually Goes Around Obstacles / Short Boxes / Holes!)
// =========================================================================

static float quick_ping_sector_min(int deg_start, int deg_end)
{
    float min_cm = 200.0f;
    int deg;
    for (deg = deg_start; deg <= deg_end; deg += 10)
    {
        servo_move((float)deg);
        timer_waitMillis(65);
        float d = ping_read_cm();
        if (d > 4.0f && d < min_cm)
        {
            min_cm = d;
        }
    }
    servo_move(90.0f);
    return min_cm;
}

// Actively sidesteps around a bumper hit, floor hole, or close scanned obstacle.
// Returns 1 if sidestep completed cleanly, 2 if destination mat hit during detour.
static int nav_detour_around_hazard(oi_t *sensor, int move_flag,
                                    float goal_x, float goal_y)
{
    float orig_bearing = bot_pose.bearing_deg;

    // 1. Back up if physically against a bump, hole, or wall
    if (move_flag != MOVE_FLAG_NONE)
    {
        move_backward_tracked(sensor, 15.0, DRIVE_SPEED, odometry_step_cb);
    }

    // 2. Measure true sideways clearance: Right (0..20 deg) vs Left (160..180 deg)
    float right_clear = quick_ping_sector_min(0, 20);
    float left_clear  = quick_ping_sector_min(160, 180);

    // Check map safety 38 cm to Right (+90 CW) vs Left (-90 CCW)
    float right_rad = DEG_TO_RAD(normalize_deg(orig_bearing + 90.0f));
    float left_rad  = DEG_TO_RAD(normalize_deg(orig_bearing - 90.0f));

    float rx = bot_pose.x + DETOUR_SIDE_CM * sinf(right_rad);
    float ry = bot_pose.y + DETOUR_SIDE_CM * cosf(right_rad);
    float lx = bot_pose.x + DETOUR_SIDE_CM * sinf(left_rad);
    float ly = bot_pose.y + DETOUR_SIDE_CM * cosf(left_rad);

    int right_map_ok = cell_is_walkable(cm_to_row(ry), cm_to_col(rx));
    int left_map_ok  = cell_is_walkable(cm_to_row(ly), cm_to_col(lx));

    // Score each side
    float right_score = right_clear + (right_map_ok ? 80.0f : -120.0f);
    float left_score  = left_clear  + (left_map_ok  ? 80.0f : -120.0f);

    // Prefer dodging away from the side that was hit
    if (move_flag == MOVE_FLAG_BUMP_LEFT  || move_flag == MOVE_FLAG_HOLE_LEFT  || move_flag == MOVE_FLAG_WALL_LEFT)
        right_score += 35.0f;
    if (move_flag == MOVE_FLAG_BUMP_RIGHT || move_flag == MOVE_FLAG_HOLE_RIGHT || move_flag == MOVE_FLAG_WALL_RIGHT)
        left_score += 35.0f;

    // Slight tie-breaker toward the goal
    float dist_r = sqrtf((goal_x - rx) * (goal_x - rx) + (goal_y - ry) * (goal_y - ry));
    float dist_l = sqrtf((goal_x - lx) * (goal_x - lx) + (goal_y - ly) * (goal_y - ly));
    if (dist_r < dist_l) right_score += 15.0f;
    else                 left_score  += 15.0f;

    int try_order[2];
    if (right_score >= left_score)
    {
        try_order[0] = 1;  // Right (+90 CW) first
        try_order[1] = -1; // Left (-90 CCW) fallback
    }
    else
    {
        try_order[0] = -1;
        try_order[1] = 1;
    }

    int attempt;
    for (attempt = 0; attempt < 2; attempt++)
    {
        int dir = try_order[attempt];
        float side_bearing = normalize_deg(orig_bearing + (float)(dir * 90));

        char buf[64];
        sprintf(buf, "DETOUR %s (R:%.0f L:%.0f)\r\n",
                (dir > 0) ? "RIGHT" : "LEFT", right_clear, left_clear);
        uart_sendStr(buf);

        // Turn 90 deg to open side
        nav_turn_to_bearing(sensor, side_bearing);

        // Check front clearance before sidestepping
        float side_fwd = quick_ping_sector_min(70, 110);
        if (side_fwd < 25.0f)
        {
            continue; // Blocked on this side, try the other side!
        }

        double step_dist = (side_fwd - 14.0f < DETOUR_SIDE_CM)
                           ? (double)(side_fwd - 14.0f) : (double)DETOUR_SIDE_CM;
        double moved = move_forward_tracked(sensor, step_dist, DRIVE_SPEED, odometry_step_cb, 0);

        if (flag == MOVE_FLAG_DESTINATION)
        {
            dest_found = 1;
            dest_x_cm = bot_pose.x;
            dest_y_cm = bot_pose.y;
            return 2;
        }
        if (flag != MOVE_FLAG_NONE)
        {
            mark_hazard_from_flag(flag);
            move_backward_tracked(sensor, 12.0, DRIVE_SPEED, odometry_step_cb);
            if (moved < 20.0)
            {
                continue; // Couldn't clear this side; try opposite side
            }
        }

        // Turn back parallel to original heading and pass the obstacle (only if map-safe!)
        nav_turn_to_bearing(sensor, orig_bearing);
        float orig_rad = DEG_TO_RAD(orig_bearing);
        float px = bot_pose.x + DETOUR_PASS_CM * sinf(orig_rad);
        float py = bot_pose.y + DETOUR_PASS_CM * cosf(orig_rad);
        float pass_clear = quick_ping_sector_min(65, 115);

        if (pass_clear > 24.0f &&
            cell_is_walkable(cm_to_row(py), cm_to_col(px)) &&
            !point_inside_any_rack(px, py, ROBOT_RADIUS_CM))
        {
            double pass_dist = (pass_clear - 15.0f < DETOUR_PASS_CM)
                               ? (double)(pass_clear - 15.0f) : (double)DETOUR_PASS_CM;
            move_forward_tracked(sensor, pass_dist, DRIVE_SPEED, odometry_step_cb, 0);

            if (flag == MOVE_FLAG_DESTINATION)
            {
                dest_found = 1;
                dest_x_cm = bot_pose.x;
                dest_y_cm = bot_pose.y;
                return 2;
            }
            if (flag != MOVE_FLAG_NONE)
            {
                mark_hazard_from_flag(flag);
                move_backward_tracked(sensor, 12.0, DRIVE_SPEED, odometry_step_cb);
            }
        }

        return 1; // Successfully sidestepped around obstacle!
    }

    return 0;
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

static int plan_bfs_path(float goal_x, float goal_y)
{
    const int total_cells = GRID_ROWS * GRID_COLS;
    const int dr[4] = { 1, 0, -1, 0 }; // N, E, S, W
    const int dc[4] = { 0, 1, 0, -1 };

    int sr = cm_to_row(bot_pose.y);
    int sc = cm_to_col(bot_pose.x);
    int gr = cm_to_row(goal_y);
    int gc = cm_to_col(goal_x);

    // If odometry drifted slightly into an inflated margin cell, escape to nearest free cell
    if (!find_nearest_walkable(sr, sc, &sr, &sc))
    {
        return 0;
    }
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

    int raw_len = 0;
    int cur = goal_idx;
    while (cur != start_idx && raw_len < MAX_PATH_LEN - 1)
    {
        raw_path[raw_len++] = (int16_t)cur;
        cur = bfs_parent[cur];
    }
    raw_path[raw_len++] = (int16_t)start_idx;

    for (i = 0; i < raw_len / 2; i++)
    {
        int16_t tmp = raw_path[i];
        raw_path[i] = raw_path[raw_len - 1 - i];
        raw_path[raw_len - 1 - i] = tmp;
    }

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
// Core Drive-To-Coordinate Loop
// =========================================================================

// Returns: 1 = reached goal, 2 = stopped on destination mat, 0 = failed
int nav_drive_to_coord(oi_t *sensor, float goal_x, float goal_y,
                       int do_scans, int stop_on_dest)
{
    int iter = 0;
    int clear_stage = 0;
    char buf[80];

    sprintf(buf, "GOAL %.1f %.1f\r\n", goal_x, goal_y);
    uart_sendStr(buf);

    while (iter < 90)
    {
        iter++;
        float dx_goal = goal_x - bot_pose.x;
        float dy_goal = goal_y - bot_pose.y;
        float dist_goal = sqrtf(dx_goal * dx_goal + dy_goal * dy_goal);

        nav_send_telemetry();
        lcd_printf("Pos:%.0f,%.0f %.0f\nGoal:%.0f,%.0f\nDist:%.0fcm",
                   bot_pose.x, bot_pose.y, bot_pose.bearing_deg,
                   goal_x, goal_y, dist_goal);

        if (dist_goal <= GOAL_TOL_CM)
        {
            uart_sendStr("ARRIVED\r\n");
            return 1;
        }

        // 1. Plan BFS path around racks, holes, bumps, and scanned pillars
        int path_len = plan_bfs_path(goal_x, goal_y);
        if (path_len == 0)
        {
            if (clear_stage == 0)
            {
                uart_sendStr("REPLAN_CLEAR_SCAN\r\n");
                clear_temp_obstacles(0); // Keep bumps & holes
                clear_stage = 1;
                continue;
            }
            else if (clear_stage == 1)
            {
                uart_sendStr("REPLAN_CLEAR_BUMPS\r\n");
                clear_temp_obstacles(1); // Keep racks, walls, holes
                clear_stage = 2;
                continue;
            }
            uart_sendStr("NO_PATH\r\n");
            return 0;
        }

        // Pick next waypoint
        float wp_x = goal_x;
        float wp_y = goal_y;
        if (path_len >= 2)
        {
            int wp_idx = smooth_path[1];
            if (path_len > 2)
            {
                wp_x = col_to_cm(wp_idx % GRID_COLS);
                wp_y = row_to_cm(wp_idx / GRID_COLS);
            }
        }

        float dx = wp_x - bot_pose.x;
        float dy = wp_y - bot_pose.y;
        float seg_dist = sqrtf(dx * dx + dy * dy);
        if (seg_dist < 3.0f)
        {
            continue;
        }

        // 2. Turn toward waypoint (encoder-primary + IMU redundancy)
        float target_bearing = normalize_deg(RAD_TO_DEG(atan2f(dx, dy)));
        nav_turn_to_bearing(sensor, target_bearing);

        float burst = (seg_dist > MAX_BURST_CM) ? MAX_BURST_CM : seg_dist;

        // 3. Scan forward hemisphere AND verify the direct driving corridor
        // (Skip full 180-deg scan on final short <26 cm approach into a standoff point)
        if (do_scans && dist_goal > 26.0f)
        {
            scan_field(0);
            mark_scanned_objects(goal_x, goal_y);

            int hit_angle = -1;
            float safe_cm = nav_check_front_corridor_cm(burst, &hit_angle);

            if (hit_angle >= 0 && safe_cm < burst)
            {
                float blocker_d = scan_fused_cm[hit_angle / 2];
                sprintf(buf, "CORRIDOR_BLOCK ang:%d d:%.1f safe:%.1f\r\n",
                        hit_angle, blocker_d, safe_cm);
                uart_sendStr(buf);

                // Mark the blocking object in the grid
                mark_corridor_blocker(hit_angle, blocker_d);

                if (safe_cm < 16.0f)
                {
                    // Too close to drive straight -> actively sidestep around it!
                    int d_res = nav_detour_around_hazard(sensor, MOVE_FLAG_NONE, goal_x, goal_y);
                    if (d_res == 2 && stop_on_dest) return 2;
                    continue;
                }
                else
                {
                    // Shorten burst so we stop safely short of the obstacle, then re-plan
                    burst = safe_cm;
                }
            }
        }

        // 4. Drive burst straight using wheel encoders
        move_forward_tracked(sensor, burst, DRIVE_SPEED, odometry_step_cb, 0);

        // 5. Handle stop triggers
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
            // Bump, hole, or wall: mark persistently & genuinely go around it!
            mark_hazard_from_flag(flag);
            clear_stage = 0;

            int d_res = nav_detour_around_hazard(sensor, flag, goal_x, goal_y);
            if (d_res == 2 && stop_on_dest)
            {
                return 2;
            }
        }
    }

    return 0;
}

// =========================================================================
// Rack Approach (Left/Right/North/South of any of the 4 racks)
// =========================================================================

int nav_approach_rack(oi_t *sensor, RackId rack, RackSide side)
{
    float tx = 0.0f, ty = 0.0f, approach_bearing = 0.0f;
    if (!nav_get_rack_target(rack, side, &tx, &ty, &approach_bearing))
    {
        uart_sendStr("INVALID_RACK_SIDE\r\n");
        return 0;
    }

    // 1. Navigate from current position to the rack side's standoff point
    if (!nav_drive_to_coord(sensor, tx, ty, 1, 0))
    {
        return 0;
    }

    // 2. Turn to face the rack wall directly
    nav_turn_to_bearing(sensor, approach_bearing);

    // 3. Measure PING straight ahead and creep to within ~6 cm of the rack
    servo_move(90.0f);
    timer_waitMillis(200);
    float d_cm = ping_read_cm();

    if (d_cm > RACK_FINAL_GAP_CM && d_cm < 45.0f)
    {
        float creep = d_cm - RACK_FINAL_GAP_CM;
        move_forward_tracked(sensor, creep, CREEP_SPEED, odometry_step_cb, 0);
        timer_waitMillis(100);
        d_cm = ping_read_cm();
    }

    // 4. Re-calibrate X or Y from known rack wall + PING distance
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
    timer_waitMillis(800);

    // 5. Back off safely to standoff before next leg
    move_backward_tracked(sensor, 14.0, DRIVE_SPEED, odometry_step_cb);
    return 1;
}

// =========================================================================
// Find Destination (Passive Hit Reuse or Unvisited-Cell Sweep)
// =========================================================================

int nav_find_destination(oi_t *sensor)
{
    if (dest_found)
    {
        uart_sendStr("DEST_KNOWN_RETURNING\r\n");
        return nav_drive_to_coord(sensor, dest_x_cm, dest_y_cm, 1, 1) > 0;
    }

    uart_sendStr("SWEEPING_FOR_DEST\r\n");
    lcd_printf("Sweeping for\ndestination...");

    const float sweep_pts[][2] = {
        {  90.0f, 131.0f },
        {  90.0f, 210.0f },
        { 262.0f, 210.0f },
        { 262.0f, 131.0f },
        { 262.0f,  55.0f },
        { 415.0f,  55.0f },
        { 415.0f, 131.0f },
        { 415.0f, 210.0f },
        {  90.0f,  55.0f }
    };
    const int num_pts = (int)(sizeof(sweep_pts) / sizeof(sweep_pts[0]));
    int i;

    for (i = 0; i < num_pts; i++)
    {
        float sx = sweep_pts[i][0];
        float sy = sweep_pts[i][1];

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

    if (!nav_approach_rack(sensor, rack1, side1))
    {
        uart_sendStr("MISSION_FAIL_RACK1\r\n");
        return 0;
    }

    if (!nav_find_destination(sensor))
    {
        uart_sendStr("MISSION_FAIL_DEST1\r\n");
        return 0;
    }
    uart_sendStr("DROPOFF_1_COMPLETE\r\n");
    timer_waitMillis(1000);

    if (!nav_approach_rack(sensor, rack2, side2))
    {
        uart_sendStr("MISSION_FAIL_RACK2\r\n");
        return 0;
    }

    if (!nav_drive_to_coord(sensor, dest_x_cm, dest_y_cm, 1, 1))
    {
        uart_sendStr("MISSION_FAIL_DEST2\r\n");
        return 0;
    }

    uart_sendStr("MISSION_COMPLETE\r\n");
    lcd_printf("ORDER COMPLETE!\nAt Packing Bay");

    unsigned char notes[4] = { 60, 64, 67, 72 };
    unsigned char dur[4]   = { 16, 16, 16, 32 };
    oi_loadSong(0, 4, notes, dur);
    oi_play_song(0);
    return 1;
}
