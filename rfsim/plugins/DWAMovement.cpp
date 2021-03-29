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

const int direction_option_count = 9;
float l_mult_possible[direction_option_count] = {-10, -5, -3, -1, 0, 1, 3, 5, 10};
float r_mult_possible[direction_option_count] = {-10, -5, -3, -1, 0, 1, 3, 5, 10};

void save_state(const rfsim_game_state_info *state);
void calculate_speeds(const rfsim_game_state_info *state, float dt, const rfsim_vec2 *team_a_speeds, const rfsim_vec2 *team_b_speeds, float *team_a_angular_velocities, float *team_b_angular_velocities);
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

double calculate_closest_obstacle_distance(const rfsim_game_state_info *state, const rfsim_vec2 &new_position) {
    double distance_to_the_closest_obstacle = 10000000000;
    for (int i = 0; i < teamSize; i += 1) {
        if (i == 0) continue;
        float deltaX = state->team_a[i].position.x - new_position.x;
        float deltaY = state->team_a[i].position.y - new_position.y;
        float distance = sqrt(deltaX * deltaX + deltaY * deltaY) - 2 * 0.15f;
        if (distance_to_the_closest_obstacle > distance) {
            distance_to_the_closest_obstacle = distance;
        }
    }
    for (int i = 0; i < teamSize; i += 1) {
        float deltaX = state->team_b[i].position.x - new_position.x;
        float deltaY = state->team_b[i].position.y - new_position.y;
        float distance = sqrt(deltaX * deltaX + deltaY * deltaY) - 2 * 0.15f;
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

rfsim_body calculate_new_position(const double vl, const double vr, const double x, const double y, const double theta, const double deltat) {
    double xnew;
    double ynew;
    double thetanew;
    // Simple special cases
    // Straight line motion
    if (vl - vr < 0.001) {
        xnew = x + vl * deltat * cos(theta);
        ynew = y + vl * deltat * sin(theta);
        thetanew = theta;
    }
    // Pure rotation motion
    else {
        float robot_radius = 0.15f;
        float robot_width = 0.3f;
        thetanew = theta + (robot_radius * deltat / (vr - vl));
        if (vl + vr < 0.001) {
            xnew = x;
            ynew = y;
        } else {
            // Rotation and arc angle of general circular motion
            // Using equations given in Lecture 2
            double totalv = vl + vr;
            xnew = x + totalv * deltat * cos(thetanew);
            ynew = y - totalv * deltat * cos(thetanew);
        }
    }
    return {{(float) xnew, (float) ynew}, (float) thetanew};
}

void move_main_robot(rfsim_game_state_info *state, rfsim_vec2 speed) {
    rfsim_vec2 robot_position = state->team_a[0].position;
    double vl = state->team_a_control[0].left_wheel_velocity;
    double vr = state->team_a_control[0].right_wheel_velocity;
    double theta = state->team_a[0].angle;
    double dt = state->dt;
    double best_benefit = -100000;
    double best_vl = vl;
    double best_vr = vr;
    rfsim_vec2 target = state->ball.position;
    double FORWARDWEIGHT = 200;
    double OBSTACLEWEIGHT = 6666;
    double ANGLEWEIGHT = 400;
    double x = robot_position.x;
    double y = robot_position.y;
    double robotMaxAcceleration = 1;
    double delta = robotMaxAcceleration * dt;
    double curr_distance_to_target = sqrt((x - target.x) * (x - target.x) + (y - target.y) * (y - target.y));
    rfsim_vec2 direction;
    direction.x = (float) cos(state->team_a->angle);
    direction.y = (float) sin(state->team_a->angle);
    double current_angle_to_target = acos(((target.x - x) * direction.x + (target.y - y) * direction.y) / length(target));

    for (int i2 = 0; i2 < direction_option_count; i2 += 1) {
        double l_mult = l_mult_possible[i2] * 5;
        for (int i1 = 0; i1 < direction_option_count; i1 += 1) {
            double r_mult = l_mult_possible[i1] * 5;

            double vl_possible = vl + l_mult * delta;
            double vr_possible = vr + r_mult * delta;

            double robotMaxVelocity = 2;
            if (vl_possible > robotMaxVelocity || vl_possible < -robotMaxVelocity || vr_possible > robotMaxVelocity || vr_possible < -robotMaxVelocity) {
                continue;
            }

            auto predicted_body = calculate_new_position(vl_possible, vr_possible, x, y, theta, dt);
            double x_predict = predicted_body.position.x;
            double y_predict = predicted_body.position.y;
            double theta_predict = predicted_body.angle;
            double pred_distance_to_target = sqrt((x_predict - target.x) * (x_predict - target.x) + (y_predict - target.y) * (y_predict - target.y));
            double distance_forward = curr_distance_to_target - pred_distance_to_target;
            // Calculate distance to closest obstacle from new position
            double distance_to_obstacle = calculate_closest_obstacle_distance(state, predicted_body.position);
            double distance_benefit = FORWARDWEIGHT * distance_forward;
            // Negative benefit: once we are less than ROBOTRADIUS from collision, linearly increasing cost
            double obstacle_benefit = 0.0;
            if (distance_to_obstacle < 0.15f) {
                obstacle_benefit = OBSTACLEWEIGHT * (0.15f - distance_to_obstacle);
            }

            rfsim_vec2 new_direction;
            new_direction.x = (float) cos(theta_predict);
            new_direction.y = (float) sin(theta_predict);
            double new_angle_to_target = acos(((target.x - x_predict) * new_direction.x + (target.y - y_predict) * new_direction.y) / length({(float)(target.x - x_predict), (float)(target.y - y_predict)}));
            double angle_benefit = ANGLEWEIGHT * abs(new_angle_to_target - current_angle_to_target);

            double benefit = distance_benefit - obstacle_benefit - angle_benefit;
            if (benefit >= best_benefit) {
                best_benefit = benefit;
                best_vl = vl_possible;
                best_vr = vr_possible;
            }
        }
    }

    state->team_a_control[0].left_wheel_velocity = (float) best_vl;
    state->team_a_control[0].right_wheel_velocity = (float) best_vr;
}

RFSIM_DEFINE_FUNCTION_TICK_GAME {
    rfsim_vec2 team_a_speeds[100];
    rfsim_vec2 team_b_speeds[100];
    float team_a_angular_velocities[100];
    float team_b_angular_velocities[100];
    calculate_speeds(state, dt, team_a_speeds, team_b_speeds, team_a_angular_velocities, team_b_angular_velocities);
    save_state(state);
    move_main_robot(state, team_a_speeds[0]);
    return rfsim_status_success;
};

RFSIM_DEFINE_FUNCTION_END_GAME {
    return rfsim_status_success;
};

RFSIM_DEFINE_FUNCTION_FINALIZE {
    return rfsim_status_success;
};
