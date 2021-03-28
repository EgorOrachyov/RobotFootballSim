#define RFSIM_EXPORTS
#include <chrono>
#include <cstring>
#include <iostream>
#include <random>
#include <rfsim/rfsim.h>

static const double MAX_MOTOR_FORCE = 100;

static int teamSize = 0;
static float robotMass = 1;
static float robotRadius;

static rfsim_body team_a_old[100];
static rfsim_body team_b_old[100];

static double team_a_target_speeds[100];
static double team_b_target_speeds[100];

const int direction_option_count = 4;
float l_mult_possible[direction_option_count] = {0, 1.5, 3, 5};
float r_mult_possible[direction_option_count] = {0, 1.5, 3, 5};

void save_state(const rfsim_game_state_info *state);
void calculate_speeds(const rfsim_game_state_info *state, float dt, const rfsim_vec2 *team_a_speeds, const rfsim_vec2 *team_b_speeds, float *team_a_angular_velocities, float *team_b_angular_velocities);
void move_obstacle_robots(const rfsim_game_state_info *state, const rfsim_vec2 *team_a_speeds, const rfsim_vec2 *team_b_speeds);
void move_main_robot(const rfsim_game_state_info *state, float dt, const rfsim_vec2 *team_a_speeds, const float *team_a_angular_velocities);
RFSIM_DEFINE_FUNCTION_INIT {
    std::strcpy(context->name, "DWA movement");
    std::strcpy(context->description, "Algorithm rewritten from python");
    return rfsim_status_success;
};

RFSIM_DEFINE_FUNCTION_BEGIN_GAME {
    teamSize = start->team_size;
    robotRadius = settings->robot_radius;
    for (int i = 0; i < teamSize; i += 1) {
        team_a_old[i] = start->team_a[i];
        team_b_old[i] = start->team_b[i];
    }

    std::default_random_engine engine(std::chrono::system_clock::now().time_since_epoch().count());
    auto dist = std::normal_distribution<float>(0, 1.0);
    for (int i = 0; i < teamSize; i += 1) {
        team_a_target_speeds[i] = dist(engine);
        team_b_target_speeds[i] = dist(engine);
    }

    return rfsim_status_success;
};

static double length(rfsim_vec2 vec) {
    return sqrt(vec.x * vec.x + vec.y * vec.y);
}

static rfsim_body calculate_new_position(rfsim_body body, rfsim_control control, rfsim_vec2 velocity, float angular_velocity, float dt) {
    float torque = (control.right_motor_force - control.left_motor_force) / 1.0f * 0.8f;
    float total_force = control.left_motor_force + control.right_motor_force;
    float new_angle = body.angle + angular_velocity * dt + torque * dt * dt / 2;
    float total_force_x = total_force * cos(new_angle);
    float total_force_y = total_force * sin(new_angle);
    float accelerationX = total_force_x / 1.0f;
    float accelerationY = total_force_y / 1.0f;
    float newX = body.position.x + velocity.x * dt + accelerationX * dt * dt / 2;
    float newY = body.position.y + velocity.y * dt + accelerationY * dt * dt / 2;
    return {{newX, newY}, new_angle};
}

double calculate_distance_to_the_closest_obstacle(const rfsim_game_state_info *state, const rfsim_vec2 &new_position) {
    double distance_to_the_closest_obstacle = 10000000000;
    for (int i = 0; i < teamSize; i += 1) {
        if (i == 0) continue;
        float deltaX = state->team_a[i].position.x - new_position.x;
        float deltaY = state->team_a[i].position.y - new_position.y;
        float distance = sqrt(deltaX * deltaX + deltaY * deltaY);
        if (distance_to_the_closest_obstacle > distance) {
            distance_to_the_closest_obstacle = distance;
        }
    }
    for (int i = 0; i < teamSize; i += 1) {
        float deltaX = state->team_b[i].position.x - new_position.x;
        float deltaY = state->team_b[i].position.y - new_position.y;
        float distance = sqrt(deltaX * deltaX + deltaY * deltaY);
        if (distance_to_the_closest_obstacle > distance) {
            distance_to_the_closest_obstacle = distance;
        }
    }
    return distance_to_the_closest_obstacle;
}

void save_state(const rfsim_game_state_info *state) {
    for (int i = 0; i < teamSize; i += 1) {
        team_a_old[i] = state->team_a[i];
        team_b_old[i] = state->team_b[i];
    }
}

void calculate_speeds(const rfsim_game_state_info *state, float dt, rfsim_vec2 *team_a_speeds, rfsim_vec2 *team_b_speeds, float *team_a_angular_velocities, float *team_b_angular_velocities) {
    for (int i = 0; i < teamSize; i += 1) {
        team_a_speeds[i].x = (state->team_a[i].position.x - team_a_old[i].position.x) / dt;
        team_a_speeds[i].y = (state->team_a[i].position.y - team_a_old[i].position.y) / dt;
        team_a_angular_velocities[i] = (state->team_a->angle - team_a_old->angle) / dt;
        team_b_angular_velocities[i] = (state->team_b->angle - team_b_old->angle) / dt;
        team_b_speeds[i].x = (state->team_b[i].position.x - team_b_old[i].position.x) / dt;
        team_b_speeds[i].y = (state->team_b[i].position.y - team_b_old[i].position.y) / dt;
    }
}

void move_obstacle_robots(rfsim_game_state_info *state, const rfsim_vec2 *team_a_speeds, const rfsim_vec2 *team_b_speeds) {
    for (int i = 0; i < teamSize; i += 1) {
        if (team_a_target_speeds[i] > length(team_a_speeds[i])) {
            state->team_a_control[i].left_motor_force = 100;
            state->team_a_control[i].right_motor_force = 100;
        } else {
            state->team_a_control[i].left_motor_force = 0;
            state->team_a_control[i].right_motor_force = 0;
        }

        if (team_b_target_speeds[i] > length(team_b_speeds[i])) {
            state->team_b_control[i].left_motor_force = 100;
            state->team_b_control[i].right_motor_force = 100;
        } else {
            state->team_b_control[i].left_motor_force = 0;
            state->team_b_control[i].right_motor_force = 0;
        }
    }
}

void move_main_robot(rfsim_game_state_info *state, float dt, rfsim_vec2 speed, float angular_velocity) {
    double best_benefit = -100000;
    float best_left_force;
    float best_right_force;
    double FORWARDWEIGHT = 200;
    double OBSTACLEWEIGHT = 6666;
    double ANGLEWEIGHT = 800;
    float x = state->team_a[0].position.x;
    float y = state->team_a[0].position.y;
    float target_x = state->ball.position.x;
    float target_y = state->ball.position.y;
    float max_speed = 1;
    if (length(speed) > max_speed) {
        state->team_a_control[0].left_motor_force = 0;
        state->team_a_control[0].right_motor_force = 0;
        return;
    }

    double curr_distance_to_target = sqrt((x - target_x) * (x - target_x) + (y - target_y) * (y - target_y));

    rfsim_vec2 direction;
    direction.x = (float) cos(state->team_a->angle);
    direction.y = (float) -sin(state->team_a->angle);
    double current_angle_to_target = (target_x * direction.x + target_y * direction.y) / length({target_x, target_y});

    for (int l_mult_index = 0; l_mult_index < direction_option_count; l_mult_index += 1) {
        float l_force = l_mult_possible[l_mult_index] * 20;
        for (int r_mult_index = 0; r_mult_index < direction_option_count; r_mult_index += 1) {
            float r_force = r_mult_possible[r_mult_index] * 20;
            rfsim_control new_control = { l_force, r_force};
            rfsim_body new_body = calculate_new_position(state->team_a[0], new_control, speed, angular_velocity, dt);
            auto new_position = new_body.position;
            auto new_angle = new_body.angle;
            // Calculate how much close we've moved to target location
            double distance_to_target = sqrt((new_position.x - target_x) * (new_position.x - target_x) + (new_position.y - target_y) * (new_position.y - target_y));
            double distance_forward = curr_distance_to_target - distance_to_target;
            double distance_benefit = FORWARDWEIGHT * distance_forward;

            double distance_to_the_closest_obstacle = calculate_distance_to_the_closest_obstacle(state, new_position);
            double obstacle_benefit = 0;
            if (distance_to_the_closest_obstacle < 0.15f) {
                obstacle_benefit = OBSTACLEWEIGHT * (0.15f - distance_to_the_closest_obstacle);
            }

            rfsim_vec2 new_direction;
            new_direction.x = (float) cos(new_angle);
            new_direction.y = (float) -sin(new_angle);
            double new_angle_to_target = (target_x * new_direction.x + target_y * new_direction.y) / length({target_x, target_y});
            double angle_benefit = ANGLEWEIGHT * abs(new_angle_to_target - current_angle_to_target);

            auto benefit = distance_benefit - obstacle_benefit - angle_benefit;
            if (benefit > best_benefit) {
                best_benefit = benefit;
                best_left_force = l_force;
                best_right_force = r_force;
            }
        }
    }

    if (best_left_force == 0 && best_right_force == 0) best_left_force = 30;
    state->team_a_control[0].left_motor_force = best_left_force;
    state->team_a_control[0].right_motor_force = best_right_force;
}
RFSIM_DEFINE_FUNCTION_TICK_GAME {
    // We want to find the best benefit where we have a positive component for closeness to target,
    // and a negative component for closeness to obstacles, for each of a choice of possible actions
    //    auto robot_position = state->team_a[0].position;
    //    auto theta = state->team_a[0].angle;
    //    float vl = state->team_a_control[0].left_motor_force;
    //    float vr = state->team_a_control[0].right_motor_force;
    //
    //
    //    auto TAU = dt * 7;


    //    std::default_random_engine engine(std::chrono::system_clock::now().time_since_epoch().count());
    //    auto dist = std::uniform_real_distribution<float>(0.0, 1.0);
    //
    //    for (int i = 0; i < teamSize; i++) {
    //        state->team_a_control[i].left_motor_force = 70.0f;
    //        state->team_a_control[i].right_motor_force = 70.0f;
    //    }
    //
    //    for (int i = 0; i < teamSize; i++) {
    //        state->team_b_control[i].left_motor_force = 70.0f;
    //        state->team_b_control[i].right_motor_force = 70.0f;
    //    }
    //
    rfsim_vec2 team_a_speeds[100];
    rfsim_vec2 team_b_speeds[100];
    float team_a_angular_velocities[100];
    float team_b_angular_velocities[100];
    calculate_speeds(state, dt, team_a_speeds, team_b_speeds, team_a_angular_velocities, team_b_angular_velocities);
    save_state(state);
    move_obstacle_robots(state, team_a_speeds, team_b_speeds);
    move_main_robot(state, dt, team_a_speeds[0], team_a_angular_velocities[0]);
    return rfsim_status_success;
};

RFSIM_DEFINE_FUNCTION_END_GAME {
    return rfsim_status_success;
};

RFSIM_DEFINE_FUNCTION_FINALIZE {
    return rfsim_status_success;
};
