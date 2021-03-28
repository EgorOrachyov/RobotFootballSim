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

#ifndef RFSIM_RFSIM_H
#define RFSIM_RFSIM_H

/**
 * The following contract is required:
 *
 * 1) The algorithm (robot control logic is implemented and compiled as shared object (library))
 * 2) The library is placed inside new simulator in order to be loaded in runtime
 * 3) User implements all required functions with defined export attributes
 * 4) On load the simulator loads this functions pointers and calls it
 *
 * The following functions must be implemented:
 *
 * Called to setup algo context and store allocated object
 * RFSIM_EXPORT RFSIM_API rfsim_status rfsim_init(rfsim_algo_state* context);
 *
 * Called when new game is started
 * RFSIM_EXPORT RFSIM_API rfsim_status rfsim_begin_game(rfsim_algo_state* context, const rfsim_game_settings* settings, const rfsim_game_start_info* start);
 *
 * Called each game tick to process update
 * RFSIM_EXPORT RFSIM_API rfsim_status rfsim_tick_game(rfsim_algo_state* context, rfsim_game_state_info* state);
 *
 * Called when game is finished
 * RFSIM_EXPORT RFSIM_API rfsim_status rfsim_end_game(rfsim_algo_state* context);
 *
 * Called to release context, free memory and etc.
 * RFSIM_EXPORT RFSIM_API rfsim_status rfsim_finalize(rfsim_algo_state* context);
 */

#ifdef __cplusplus
    #include <cinttypes>
#else
    #include <inttypes.h>
#endif

// Preserve C names in shared library
#ifdef __cplusplus
    #define RFSIM_EXPORT extern "C"
#else
    #define RFSIM_EXPORT
#endif

#if (_MSC_VER && !__INTEL_COMPILER)
    #ifdef RFSIM_EXPORTS
        #define RFSIM_API __declspec(dllexport)
    #else
        #define RFSIM_API __declspec(dllimport)
    #endif
#else
    // Default case
    #define RFSIM_API
#endif

/** Global consts goes here */

#define RFSIM_MAX_ROBOTS_PER_TEAM 100
#define RFSIM_MAX_SHORT_STRING_LENGTH 256
#define RFSIM_MAX_LONG_STRING_LENGTH 512

/** Defined data structures */

typedef struct rfsim_vec2 {
    float x;
    float y;
} rfsim_vec2;

typedef struct rfsim_body {
    rfsim_vec2 position;
    float angle;
} rfsim_body;

typedef struct rfsim_control {
    float left_motor_force;
    float right_motor_force;
} rfsim_control;

typedef struct rfsim_game_settings {
    rfsim_vec2 field_Size;
    float robot_radius;
    float ball_radius;
} rfsim_game_settings;

typedef struct rfsim_game_start_info {
    rfsim_body team_a[RFSIM_MAX_ROBOTS_PER_TEAM];
    rfsim_body team_b[RFSIM_MAX_ROBOTS_PER_TEAM];
    rfsim_body ball;
    int team_size;
} rfsim_game_start_info;

typedef struct rfsim_game_state_info {
    rfsim_body team_a[RFSIM_MAX_ROBOTS_PER_TEAM];
    rfsim_body team_b[RFSIM_MAX_ROBOTS_PER_TEAM];
    rfsim_control team_a_control[RFSIM_MAX_ROBOTS_PER_TEAM];
    rfsim_control team_b_control[RFSIM_MAX_ROBOTS_PER_TEAM];
    rfsim_body ball;
} rfsim_game_state_info;

typedef struct rfsim_algo_state {
    char name[RFSIM_MAX_SHORT_STRING_LENGTH];
    char description[RFSIM_MAX_LONG_STRING_LENGTH];
    void* user_data;
} rfsim_algo_state;

/** Operation status, returned by each exported function */
typedef enum rfsim_status {
    rfsim_status_success,
    rfsim_status_error
} rfsim_status;

/*
 * Functions names
 */

#define RFSIM_FUNCTION_INIT_NAME "rfsim_init"
#define RFSIM_FUNCTION_BEGIN_GAME_NAME "rfsim_begin_game"
#define RFSIM_FUNCTION_TICK_GAME_NAME "rfsim_tick_game"
#define RFSIM_FUNCTION_END_GAME_NAME "rfsim_end_game"
#define RFSIM_FUNCTION_FINALIZE_NAME "rfsim_finalize"

/*
 * Functions types
 */

typedef rfsim_status (*rfsim_init_type)(rfsim_algo_state* context);
typedef rfsim_status (*rfsim_begin_game_type)(rfsim_algo_state* context, const rfsim_game_settings* settings, const rfsim_game_start_info* start);
typedef rfsim_status (*rfsim_tick_game_type)(rfsim_algo_state* context, rfsim_game_state_info* state, float dt);
typedef rfsim_status (*rfsim_end_game_type)(rfsim_algo_state* context);
typedef rfsim_status (*rfsim_finalize_type)(rfsim_algo_state* context);

/*
 * Use these macro to define required functions in your algo plugin.
 */

#define RFSIM_DEFINE_FUNCTION_INIT \
    RFSIM_EXPORT RFSIM_API rfsim_status rfsim_init(rfsim_algo_state* context)

#define RFSIM_DEFINE_FUNCTION_BEGIN_GAME \
    RFSIM_EXPORT RFSIM_API rfsim_status rfsim_begin_game(rfsim_algo_state* context, const rfsim_game_settings* settings, const rfsim_game_start_info* start)

#define RFSIM_DEFINE_FUNCTION_TICK_GAME \
    RFSIM_EXPORT RFSIM_API rfsim_status rfsim_tick_game(rfsim_algo_state* context, rfsim_game_state_info* state, float dt)

#define RFSIM_DEFINE_FUNCTION_END_GAME \
    RFSIM_EXPORT RFSIM_API rfsim_status rfsim_end_game(rfsim_algo_state* context)

#define RFSIM_DEFINE_FUNCTION_FINALIZE \
    RFSIM_EXPORT RFSIM_API rfsim_status rfsim_finalize(rfsim_algo_state* context)

#endif //RFSIM_RFSIM_H
