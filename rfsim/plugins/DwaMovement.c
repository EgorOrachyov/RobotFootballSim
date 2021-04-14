////////////////////////////////////////////////////////////////////////////////////
// MIT License                                                                    //
//                                                                                //
// Copyright (c) 2021 The RobotFootballSim project authors                        //
//                                                                                //
// Permission is hereby granted, free of charge, to any person obtaining a copy   //
// of this software and associated documentation files (the "Software"), to deal  //
// in the Software without restriction, including without limitation the rights   //
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell      //
// copies of the Software, and to permit persons to whom the Software is          //
// furnished to do so, subject to the following conditions:                       //
//                                                                                //
// The above copyright notice and this permission notice shall be included in all //
// copies or substantial portions of the Software.                                //
//                                                                                //
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR     //
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,       //
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE    //
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER         //
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,  //
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE  //
// SOFTWARE.                                                                      //
////////////////////////////////////////////////////////////////////////////////////

#define RFSIM_EXPORTS
#include <rfsim/rfsim.h>

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/**
 * @file DwaMovement.c
 *
 * @brief **Dynamic Window Approach** collision avoidance algorithm implementation
 *
 * @details The algorithm is probably the simplest possible and the most customizable one.
 * Imagine the moving robot. How can it change its movement? It can change its linear velocity
 * with a finite amount of variants `V` (because robot has maximum possible acceleration) and,
 * similarly, its angular velocity with a finite amount of variants `W`.
 * So we have a 2D table of `V * W` possible movement changes, among which we should take
 * one of the most preferable according to some criteria. After some time `PREDICT_TIME` we repeat
 * the procedure to refine the direction again. That is it, nothing more.
 *
 * Which criteria to use and how to juxtapose them is up to you. Authors of the algorithm suggest the following:
 * - **Clearance** criterion (recall it is a collision avoidance algorithm, right?):
 *     we want to keep a distance from other robots. So we calculate the minimum distance from other robots
 *     for each possible velocity and the less than the robot 's radius it is, the more penalty is.
 *     @link calc_clearance_cost(rfsim_body,dwa_velocity,const rfsim_body_list*,const dwa_config*)
 *
 * - **Heading** criterion (recall we want to get to the destination, right?):
 *     there are some variations of this criterion, but the most simple is just euclidean metric:
 *     the closer we get to the target, the lesser the penalty is.
 *     @link calc_heading_cost(rfsim_body,rfsim_vec2)
 *
 * - **Velocity** criterion (recall we want to get to the destination as fast as possible, right?):
 *     the higher the velocity is, the lesser penalty is.
 *     @link calc_velocity_cost(dwa_velocity,dwa_velocity,const dwa_config*)
 *
 * Traditionally, criteria are combined using a plain _weighted sum_. But for weights to be meaningful,
 * each criterion should be normalized, e.g. to take values from `[0; 1]` each.
 * Otherwise, if the first criterion is from `[-15; 650]`, second from `[30; 40]` and third from `[0; 0.1]`,
 * what does it mean at all?
 *
 * To sum up, you can customize ranges which are taken under consideration,
 * criteria and the way to combine them. It is a very hackable and simple algorithm.
 *
 * @see plan(rfsim_body,dwa_velocity,rfsim_vec2,const rfsim_body_list*,const dwa_config*)
 *      Algorithm main planning procedure
 * @see https://github.com/goktug97/DynamicWindowApproach Original implementation
 * @see https://github.com/amslabtech/dwa_planner         Visualization of the algorithm
 */

#define FIELD_OF(type, member) (((type*) 0)->member)

//#define MY_DEBUG

#ifdef MY_DEBUG
#define DEBUG_(x) (x)
#else
#define DEBUG_(x)                                                                                            \
    do {                                                                                                     \
    } while (0)
#endif

float norm(float x, float y) {
    return sqrtf(x * x + y * y);
}

float dist(rfsim_vec2 p1, rfsim_vec2 p2) {
    const float dx = p1.x - p2.x;
    const float dy = p1.y - p2.y;
    return sqrtf(dx * dx + dy * dy);
}

static rfsim_game_settings GAME_SETTINGS;

static int   TEAM_SIZE;
static float FIELD_DIAG;

#define EPS 1e-4f

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ---------------------------------------------------------------------------------
//region Data structures

typedef struct {
    int         size;
    rfsim_body* bodies;
} rfsim_body_list;

/**
 * Contains various configurations of the algorithm.
 */
typedef struct {
    /** maximum linear speed robot can reach [m/s] */
    float max_speed;

    /** minimum linear speed robot can fall [m/s] */
    float min_speed;

    /** maximum angular speed robot can reach [yaw/s] */
    float max_yaw_speed;

    /** maximum linear acceleration robot can reach [m/ss] */
    float max_lin_accel;

    /** maximum angular acceleration robot can reach [yaw/ss] */
    float max_yaw_accel;

    /** linear speed resolution [m/s] */
    float lin_speed_resolution;

    /** angular speed resolution [m/s] */
    float yaw_speed_resolution;

    /** time step [s] */
    float dt;

    /** simulation time [s] */
    float predict_time;

    /** DWA heading cost weight */
    float heading_cost_weight;

    /** DWA clearance (obstacles) cost weight */
    float clearance_cost_weight;

    /** DWA dwa_velocity cost weight */
    float velocity_cost_weight;
} dwa_config;

typedef struct {
    float linear;
    float angular;
} dwa_velocity;

/**
 * 2D Table of possible velocity variants.
 */
typedef struct {
    int    possible_v_size;
    float* possible_v;

    int    possible_w_size;
    float* possible_w;
} dwa_dynamic_window;


static dwa_config DWA_CONFIG = {
        .max_speed             = 0.0f,// is set later from simulator settings
        .min_speed             = 0.0f,
        .max_yaw_speed         = (float) (360.0 * M_PI / 180.0),
        .max_lin_accel         = 0.3f,
        .max_yaw_accel         = (float) (360.0 * M_PI / 180.0),
        .lin_speed_resolution  = 0.005f,// must be <= max_lin_accel * dt
        .yaw_speed_resolution  = (float) (2.0 * M_PI / 180.0),
        .dt                    = 0.1f,
        .predict_time          = 2.0f,
        .heading_cost_weight   = 0.8f,
        .clearance_cost_weight = 1.0f,
        .velocity_cost_weight  = 0.1f,
};

//endregion
// ---------------------------------------------------------------------------------


// ---------------------------------------------------------------------------------
//region Function declarations

dwa_dynamic_window* new_dynamic_window(dwa_velocity velocity, const dwa_config* config);
void                free_dynamic_window(dwa_dynamic_window* window);

rfsim_body_list* new_obstacle_list(int size);
void             free_obstacle_list(rfsim_body_list* obstacles);

rfsim_body move_body(rfsim_body body, dwa_velocity velocity, float dt);

float calc_velocity_cost(dwa_velocity source, dwa_velocity target, const dwa_config* config);
float calc_heading_cost(rfsim_body source, rfsim_vec2 target);
float calc_clearance_cost(
        rfsim_body             rfsim_body,
        dwa_velocity           velocity,
        const rfsim_body_list* obstacles,
        const dwa_config*      config);

dwa_velocity plan(
        rfsim_body             source,
        dwa_velocity           source_velocity,
        rfsim_vec2             target,
        const rfsim_body_list* obstacles,
        const dwa_config*      config);
//endregion
// ---------------------------------------------------------------------------------


// ---------------------------------------------------------------------------------
//region DWA function definitions

/**
 * Simple quick check that robot can move with such velocity.
 */
int is_velocity_adequate(dwa_velocity v) {
    return DWA_CONFIG.min_speed <= v.linear && v.linear <= DWA_CONFIG.max_speed &&
           fabsf(v.angular) <= DWA_CONFIG.max_yaw_speed;
}

/**
 * Helps to escape accuracy lost when dividing very small value.
 */
float safe_divf(float dividend, float divisor) {
    return fabsf(dividend) < EPS ? 0.0f : dividend / divisor;
}

/**
 * Creates new table of possible velocity variants for algorithm to consider.
 */
dwa_dynamic_window* new_dynamic_window(dwa_velocity velocity, const dwa_config* config) {
    assert(is_velocity_adequate(velocity));

    const float min_v = fmaxf(config->min_speed, velocity.linear - config->max_lin_accel * config->dt);
    const float max_v = fminf(config->max_speed, velocity.linear + config->max_lin_accel * config->dt);
    const float min_w = fmaxf(-config->max_yaw_speed, velocity.angular - config->max_yaw_accel * config->dt);
    const float max_w = fminf(config->max_yaw_speed, velocity.angular + config->max_yaw_accel * config->dt);

    assert(is_velocity_adequate((dwa_velocity){.linear = min_v, .angular = min_w}));
    assert(is_velocity_adequate((dwa_velocity){.linear = max_v, .angular = max_w}));

    const int possible_v_size = (int) floorf(safe_divf(max_v - min_v, config->lin_speed_resolution));
    const int possible_w_size = (int) floorf(safe_divf(max_w - min_w, config->yaw_speed_resolution));

    dwa_dynamic_window* window = malloc(sizeof(dwa_dynamic_window));

    window->possible_v_size = possible_v_size;
    window->possible_w_size = possible_w_size;
    window->possible_v      = malloc(possible_v_size * sizeof(*FIELD_OF(dwa_dynamic_window, possible_v)));
    window->possible_w      = malloc(possible_w_size * sizeof(*FIELD_OF(dwa_dynamic_window, possible_w)));

    for (int i = 0; i < possible_v_size; ++i) {
        window->possible_v[i] = min_v + (float) i * config->lin_speed_resolution;
    }

    for (int i = 0; i < possible_w_size; ++i) {
        window->possible_w[i] = min_w + (float) i * config->yaw_speed_resolution;
    }

    return window;
}

/**
 * Frees the table of possible velocity variants.
 */
void free_dynamic_window(dwa_dynamic_window* window) {
    free(window->possible_v);
    free(window->possible_w);
    free(window);
}

/**
 * Creates new list of obstacles (typically other robots) for
 * @link calc_clearance_cost(rfsim_body, dwa_velocity, const rfsim_body_list*, const dwa_config*)
 */
rfsim_body_list* new_obstacle_list(int size) {
    rfsim_body_list* obstacles = malloc(sizeof(rfsim_body_list));
    obstacles->size            = size;
    obstacles->bodies          = malloc(size * sizeof(*FIELD_OF(rfsim_body_list, bodies)));
    return obstacles;
}

/**
 * Frees the list of obstacles.
 */
void free_obstacle_list(rfsim_body_list* obstacles) {
    free(obstacles->bodies);
    free(obstacles);
}

/**
 * Calculates target position where the body will be
 * if it moves with given `velocity` during `dt` time.
 */
rfsim_body move_body(rfsim_body body, dwa_velocity velocity, float dt) {
    assert(is_velocity_adequate(velocity));

    const float new_angle = body.angle + velocity.angular * dt;
    return (rfsim_body){
            .angle    = new_angle,
            .position = {
                    .x = body.position.x + velocity.linear * cosf(new_angle) * dt,
                    .y = body.position.y + velocity.linear * sinf(new_angle) * dt,
            }};
}

/**
 * Calculates velocity cost, see main algorithm description.
 */
float calc_velocity_cost(dwa_velocity source, dwa_velocity target, const dwa_config* config) {
    assert(is_velocity_adequate(source));
    assert(is_velocity_adequate(target));

    return 1 - target.linear / config->max_speed;
    //    const float max_possible = fminf(config->max_speed - source.linear, config->dt * config->max_lin_accel);
    //    return ((1 - target.linear / config->max_speed) +
    //            (1 - fmaxf(0, target.linear - source.linear) / max_possible)) /
    //           2;
}

/**
 * Calculates heading cost, see main algorithm description.
 */
float calc_heading_cost(rfsim_body source, rfsim_vec2 target) {
    //            1 (simple distance)
    return dist(source.position, target) / FIELD_DIAG;

    //        2 (angle error)
    //    const float dx           = target.x - source.position.x;
    //    const float dy           = target.y - source.position.y;
    //    const float wanted_angle = atan2f(dy, dx);
    //    const float angle_diff   = wanted_angle - source.angle;
    //    const float angle_cost   = fabsf(atan2f(sinf(angle_diff), cosf(angle_diff)));
    //    return angle_cost / M_PI;
}

/**
 * Calculates clearance cost, see main algorithm description.
 */
float calc_clearance_cost(
        rfsim_body             body,
        dwa_velocity           velocity,
        const rfsim_body_list* obstacles,
        const dwa_config*      config) {

    assert(is_velocity_adequate(velocity));

    float      min_r     = FLT_MAX;
    rfsim_body curr_pose = body;
    for (float time = 0.0f; time < config->predict_time; time += config->dt) {// NOLINT(cert-flp30-c)
        curr_pose = move_body(curr_pose, velocity, config->dt);

        for (int i = 0; i < obstacles->size; ++i) {
            const float d = dist(curr_pose.position, obstacles->bodies[i].position);
            const float r = fmaxf(0.0f, d - 2 * GAME_SETTINGS.robot_radius);
            if (r < min_r) {
                min_r = r;
            }
        }
    }
    return fabsf(min_r) < EPS ? FLT_MAX
                              : fmaxf(0, GAME_SETTINGS.robot_radius - min_r) / GAME_SETTINGS.robot_radius;
}

/**
 * Main DWA algorithm planning procedure:
 * 1. creates table of possible variants
 * 2. iterates over it and counts criteria
 * 3. selects the best velocity variant with the least cost
 */
dwa_velocity plan(
        rfsim_body             source,
        dwa_velocity           source_velocity,
        rfsim_vec2             target,
        const rfsim_body_list* obstacles,
        const dwa_config*      config) {

    assert(is_velocity_adequate(source_velocity));

    dwa_dynamic_window* window = new_dynamic_window(source_velocity, config);

    float        best_cost           = FLT_MAX;
    float        best_velocity_cost  = FLT_MAX;
    float        best_heading_cost   = FLT_MAX;
    float        best_clearance_cost = FLT_MAX;
    dwa_velocity best_velocity       = { 0, 0 };

    for (int i = 0; i < window->possible_v_size; ++i) {
        for (int j = 0; j < window->possible_w_size; ++j) {
            const dwa_velocity target_velocity = {
                    .linear  = window->possible_v[i],
                    .angular = window->possible_w[j],
            };
            assert(is_velocity_adequate(target_velocity));

            const rfsim_body target_pose = move_body(source, target_velocity, config->predict_time);

            const float velocity_cost  = calc_velocity_cost(source_velocity, target_velocity, config);
            const float heading_cost   = calc_heading_cost(target_pose, target);
            const float clearance_cost = calc_clearance_cost(source, target_velocity, obstacles, config);

            const float target_cost = config->velocity_cost_weight * velocity_cost +
                                      config->heading_cost_weight * heading_cost +
                                      config->clearance_cost_weight * clearance_cost;

            if (target_cost < best_cost) {
                best_cost           = target_cost;
                best_velocity_cost  = velocity_cost;
                best_heading_cost   = heading_cost;
                best_clearance_cost = clearance_cost;
                best_velocity       = target_velocity;
            }
        }
    }

    DEBUG_(
            printf("TC = % 2.3f | CC = % 2.3f | HC = % 2.3f | VC = % 2.3f | ",
                   best_cost,
                   best_clearance_cost,
                   best_heading_cost,
                   best_velocity_cost));

    assert(is_velocity_adequate(best_velocity));
    free_dynamic_window(window);
    return best_velocity;
}
//endregion
// ---------------------------------------------------------------------------------


// ---------------------------------------------------------------------------------
//region Game functions definitions

float uniform_rand(float min, float max) {
    double scale = rand() / (double) RAND_MAX;// NOLINT(cert-msc50-cpp)
    return (float) (min + scale * (max - min));
}

enum algo_state { MOVING_TO_HIT_POSITION, CALIBRATING_ANGLE, READY_HIT };

static enum algo_state ALGO_STATE = MOVING_TO_HIT_POSITION;

RFSIM_DEFINE_FUNCTION_INIT {
    strcpy(context->name, "DWA Ball Follow - Random Obstacles");
    strcpy(context->description, "Single robot eternally follows the ball among randomly moving robots");

    srand(time(NULL));// NOLINT(cert-msc51-cpp)

    return rfsim_status_success;
}

RFSIM_DEFINE_FUNCTION_BEGIN_GAME {
    GAME_SETTINGS        = *settings;
    TEAM_SIZE            = start->team_size;
    FIELD_DIAG           = norm(settings->field_Size.x, settings->field_Size.y);
    DWA_CONFIG.max_speed = settings->robot_max_speed;
    ALGO_STATE           = MOVING_TO_HIT_POSITION;
    return rfsim_status_success;
}

void set_random_control(rfsim_control* control, bool to_left) {
    control->left_wheel_velocity  = uniform_rand(0.0f, 0.5f) * (to_left ? 1.5f : 1.0f);
    control->right_wheel_velocity = uniform_rand(0.0f, 0.5f) * (to_left ? 1.0f : 1.5f);
}

int signf(float x) {
    return (x > 0) - (x < 0);
}

float clampf(float v, float min, float max) {
    return fmaxf(fminf(v, max), min);
}

/**
 * @see http://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf
 */
rfsim_control velocity_dwa_to_rfsim(dwa_velocity v, float dt) {
    assert(is_velocity_adequate(v));

    const float l = GAME_SETTINGS.robot_wheel_x_offset * 2;

    const float w = -v.angular;

    if (1 - cosf(w * dt) < 1e-6) {
        return (rfsim_control){
                .left_wheel_velocity  = v.linear,
                .right_wheel_velocity = v.linear,
        };
    } else {
        const float R = sqrtf(v.linear * dt * v.linear * dt / 2.0f / (1 - cosf(w * dt))) * ((float) signf(w));

        const float rw = (w * l + 2 * R * w) / 2.0f;
        const float lw = 2 * R * w - rw;

        return (rfsim_control){
                .left_wheel_velocity  = lw,
                .right_wheel_velocity = rw,
        };
    }
}

/**
 * @see http://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf
 */
dwa_velocity velocity_rfsim_to_dwa(rfsim_control v, float dt) {
    const float v_l = v.left_wheel_velocity;
    const float v_r = v.right_wheel_velocity;

    const float l = GAME_SETTINGS.robot_wheel_x_offset * 2;

    dwa_velocity result;
    if (fabsf(v_l - v_r) < EPS) {
        result = (dwa_velocity){
                .linear  = (v_r + v_l) / 2.0f,
                .angular = 0.0f,
        };
    } else {
        const float R = l / 2.0f * (v_l + v_r) / (v_l - v_r);
        const float w = (v_l - v_r) / l;

        // we suppose x = 0, y = 0, sin_theta = 0, cos_theta = 1
        // then we get following simplified formulas
        const float x_n = sinf(w * dt) * R;
        const float y_n = cosf(w * dt) * (-R) + R;

        result = (dwa_velocity){
                .linear  = clampf(norm(x_n, y_n) / dt, DWA_CONFIG.min_speed, DWA_CONFIG.max_speed),
                .angular = clampf(w, -DWA_CONFIG.max_yaw_speed, DWA_CONFIG.max_yaw_speed),
        };
    }

    assert(is_velocity_adequate(result));
    return result;
}

rfsim_control dwa_move_to_target(
        rfsim_control          initial_control,
        rfsim_body             initial_position,
        const float            dt,
        const rfsim_body_list* obstacles,
        rfsim_vec2             target) {

    dwa_velocity  initial_v = velocity_rfsim_to_dwa(initial_control, dt);
    dwa_velocity  result_v  = plan(initial_position, initial_v, target, obstacles, &DWA_CONFIG);
    rfsim_control converted = velocity_dwa_to_rfsim(result_v, dt);

    DEBUG_(
            printf("L = % 2.3f | W = % 2.3f | LW = % 2.3f | RW = % 2.3f \n",
                   result_v.linear,
                   result_v.angular,
                   converted.left_wheel_velocity,
                   converted.right_wheel_velocity));
    return converted;
}

rfsim_control rotate_to_angle(float source_angle, float target_angle, float dt) {
    return velocity_dwa_to_rfsim(
            (dwa_velocity){
                    .linear = 0.0f,
                    .angular =
                            clampf((target_angle - source_angle) / dt,
                                   -DWA_CONFIG.max_yaw_speed / 4,
                                   DWA_CONFIG.max_yaw_speed / 4)},
            dt);
}

RFSIM_DEFINE_FUNCTION_TICK_GAME {
    //     set all robots except team_a[0] to move randomly
    set_random_control(&state->team_b_control[0], true);
    for (int i = 1; i < TEAM_SIZE; ++i) {
        set_random_control(&state->team_a_control[i], true);
        set_random_control(&state->team_b_control[i], false);
    }

    // consider all robots as obstacles for team_a[0] DWA
    static rfsim_body     obstacle_bodies[RFSIM_MAX_ROBOTS_PER_TEAM * 2 - 1];
    const rfsim_body_list obstacles = {
            .size   = TEAM_SIZE * 2 - 1,
            .bodies = obstacle_bodies,
    };
    memcpy(obstacle_bodies, &state->team_a[1], (TEAM_SIZE - 1) * sizeof(*state->team_a));
    memcpy(&obstacle_bodies[TEAM_SIZE - 1], state->team_b, TEAM_SIZE * sizeof(*state->team_b));

    static const rfsim_vec2 BEFORE_HIT_POSITION = {.x = 5.85f, .y = 5.4f};
    static const float      BEFORE_HIT_ANGLE    = (float) -0.7175;
    static const rfsim_vec2 TARGET_FOR_HIT      = {.x = 9.0f, .y = 3.0f};

    const rfsim_body* forward     = &state->team_a[0];
    rfsim_control*    forward_ctl = &state->team_a_control[0];

    switch (ALGO_STATE) {
        case MOVING_TO_HIT_POSITION:
            if (dist(forward->position, BEFORE_HIT_POSITION) > 0.1) {
                *forward_ctl = dwa_move_to_target(
                        *forward_ctl,
                        *forward,
                        state->dt,
                        &obstacles,
                        BEFORE_HIT_POSITION);
                break;
            }
        case CALIBRATING_ANGLE:
            ALGO_STATE = CALIBRATING_ANGLE;
            if (fabsf(forward->angle - BEFORE_HIT_ANGLE) > 0.01) {
                *forward_ctl = rotate_to_angle(forward->angle, BEFORE_HIT_ANGLE, state->dt);
                break;
            }
        case READY_HIT:
            ALGO_STATE   = READY_HIT;
            *forward_ctl = dwa_move_to_target(*forward_ctl, *forward, state->dt, &obstacles, TARGET_FOR_HIT);
            break;
        default:
            assert(false);
    }

    return rfsim_status_success;
}

RFSIM_DEFINE_FUNCTION_END_GAME {
    return rfsim_status_success;
}

RFSIM_DEFINE_FUNCTION_FINALIZE {
    return rfsim_status_success;
}
//endregion
// ---------------------------------------------------------------------------------