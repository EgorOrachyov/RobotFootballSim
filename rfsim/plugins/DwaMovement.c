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

// based on the https://github.com/goktug97/DynamicWindowApproach

#define FIELD_OF(type, member) (((type*) 0)->member)

//#define MY_DEBUG

#ifdef MY_DEBUG
#define DEBUG_(x) (x)
#else
#define DEBUG_(x)                                                                                            \
    do {                                                                                                     \
    } while (0)
#endif

static int   TEAM_SIZE    = 0;
static float FIELD_WIDTH  = 0;
static float FIELD_HEIGHT = 0;
static float FIELD_DIAG   = 0;
static float ROBOT_RADIUS = 0;
static float DT           = 0;

// TODO: where from should one get it?
#define WHEEL_RADIUS 0.15f

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

typedef struct {
    int    possible_v_size;
    float* possible_v;

    int    possible_w_size;
    float* possible_w;
} dwa_dynamic_window;


const static dwa_config DWA_CONFIG = {
        .max_speed             = 1.0f,
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

int is_velocity_adequate(dwa_velocity v) {
    return DWA_CONFIG.min_speed <= v.linear && v.linear <= DWA_CONFIG.max_speed &&
           fabsf(v.angular) <= DWA_CONFIG.max_yaw_speed;
}

float safe_divf(float dividend, float divisor) {
    return fabsf(dividend) < EPS ? 0.0f : dividend / divisor;
}

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

void free_dynamic_window(dwa_dynamic_window* window) {
    free(window->possible_v);
    free(window->possible_w);
    free(window);
}

rfsim_body_list* new_obstacle_list(int size) {
    rfsim_body_list* obstacles = malloc(sizeof(rfsim_body_list));
    obstacles->size            = size;
    obstacles->bodies          = malloc(size * sizeof(*FIELD_OF(rfsim_body_list, bodies)));
    return obstacles;
}

void free_obstacle_list(rfsim_body_list* obstacles) {
    free(obstacles->bodies);
    free(obstacles);
}

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

float calc_velocity_cost(dwa_velocity source, dwa_velocity target, const dwa_config* config) {
    assert(is_velocity_adequate(source));
    assert(is_velocity_adequate(target));

    return 1 - target.linear / config->max_speed;
    //    const float max_possible = fminf(config->max_speed - source.linear, config->dt * config->max_lin_accel);
    //    return ((1 - target.linear / config->max_speed) +
    //            (1 - fmaxf(0, target.linear - source.linear) / max_possible)) /
    //           2;
}

float calc_heading_cost(rfsim_body source, rfsim_vec2 target) {
    //            1 (simple distance)
    const float dx = target.x - source.position.x;
    const float dy = target.y - source.position.y;
    return sqrtf(dx * dx + dy * dy) / FIELD_DIAG;

    //        2 (angle error)
    //    const float dx           = target.x - source.position.x;
    //    const float dy           = target.y - source.position.y;
    //    const float wanted_angle = atan2f(dy, dx);
    //    const float angle_diff   = wanted_angle - source.angle;
    //    const float angle_cost   = fabsf(atan2f(sinf(angle_diff), cosf(angle_diff)));
    //    return angle_cost / M_PI;
}

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
            const float dx = curr_pose.position.x - obstacles->bodies[i].position.x;
            const float dy = curr_pose.position.y - obstacles->bodies[i].position.y;

            const float r = fmaxf(0.0f, sqrtf(dx * dx + dy * dy) - 2 * ROBOT_RADIUS);
            if (r < min_r) {
                min_r = r;
            }
        }
    }
    return fabsf(min_r) < EPS ? FLT_MAX : fmaxf(0, ROBOT_RADIUS - min_r) / ROBOT_RADIUS;
}

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
    dwa_velocity best_velocity       = {};

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

RFSIM_DEFINE_FUNCTION_INIT {
    strcpy(context->name, "DWA Ball Follow - Random Obstacles");
    strcpy(context->description, "Single robot eternally follows the ball among randomly moving robots");

    srand(time(NULL));// NOLINT(cert-msc51-cpp)

    return rfsim_status_success;
}

RFSIM_DEFINE_FUNCTION_BEGIN_GAME {
    TEAM_SIZE    = start->team_size;
    FIELD_HEIGHT = settings->field_Size.y;
    FIELD_WIDTH  = settings->field_Size.x;
    FIELD_DIAG   = sqrtf(FIELD_HEIGHT * FIELD_HEIGHT + FIELD_WIDTH * FIELD_WIDTH);
    ROBOT_RADIUS = settings->robot_radius;
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
rfsim_control velocity_dwa_to_rfsim(dwa_velocity v) {
    assert(is_velocity_adequate(v));

    const float l = WHEEL_RADIUS * 2;

    const float w = -v.angular;

    if (1 - cosf(w * DT) < 1e-6) {
        return (rfsim_control){
                .left_wheel_velocity  = v.linear,
                .right_wheel_velocity = v.linear,
        };
    } else {
        const float R = sqrtf(v.linear * DT * v.linear * DT / 2.0f / (1 - cosf(w * DT))) * ((float) signf(w));

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
dwa_velocity velocity_rfsim_to_dwa(rfsim_control v) {
    const float v_l = v.left_wheel_velocity;
    const float v_r = v.right_wheel_velocity;

    const float l = WHEEL_RADIUS * 2;

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
        const float x_n = sinf(w * DT) * R;
        const float y_n = cosf(w * DT) * (-R) + R;

        result = (dwa_velocity){
                .linear =
                        clampf(sqrtf(x_n * x_n + y_n * y_n) / DT, DWA_CONFIG.min_speed, DWA_CONFIG.max_speed),
                .angular = clampf(w, -DWA_CONFIG.max_yaw_speed, DWA_CONFIG.max_yaw_speed),
        };
    }

    assert(is_velocity_adequate(result));
    return result;
}

RFSIM_DEFINE_FUNCTION_TICK_GAME {
    DT = state->dt;

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

    // run DWA
    dwa_velocity velocity = velocity_rfsim_to_dwa(state->team_a_control[0]);
    dwa_velocity result_v = plan(state->team_a[0], velocity, state->ball.position, &obstacles, &DWA_CONFIG);

    rfsim_control converted = velocity_dwa_to_rfsim(result_v);

    DEBUG_(
            printf("L = % 2.3f | W = % 2.3f | LW = % 2.3f | RW = % 2.3f \n",
                   result_v.linear,
                   result_v.angular,
                   converted.left_wheel_velocity,
                   converted.right_wheel_velocity));

    state->team_a_control[0] = converted;

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