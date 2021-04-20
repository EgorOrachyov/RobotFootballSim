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
#include <stdlib.h>
#include <cstring>
#include <list>
#include <deque>
#include <limits>
#include <cmath>
#include <iostream>
#include <float.h>

static rfsim_game_settings g_settings;
static rfsim_game_start_info g_start_info;

static bool flag = true;

#define EPS 1e-4f

#define M_PI 3.14159265358979323846

static float FIELD_DIAG;

#define FIELD_OF(type, member) (((type*) 0)->member)

static double getLength(const rfsim_vec2& v) {
    return sqrt(pow(v.x, 2) + pow(v.y, 2));
}

static double getDistance(const rfsim_vec2& v1, const rfsim_vec2& v2) {
    return sqrt(pow(v1.x - v2.x, 2) + pow(v1.y - v2.y, 2));
}

struct PlayerInfo {
    int team;
    int id;
};

class Node {
    public:
        Node(const rfsim_vec2& state, Node* parent)
            : _state(state), _parent(parent) {
            if (_parent) {
                _parent->_children.push_back(this);
            }
        }

        rfsim_vec2 state() { return _state; };

        const Node* parent() { return _parent; };

    private:
        rfsim_vec2 _state;
        Node* _parent;
        std::list<Node*> _children;
};

class Tree {
    //создать дерево, положить в него состояние поля, инфу об игроке (см структуру)
    //вызвать run, если хорошо, то вызвать getPath. первый элемент - ближайшая точка
    public:
        Tree(rfsim_game_state_info* gameState, PlayerInfo playerInfo) {
            _gameState = gameState;
            _playerInfo = playerInfo;
            _startState = playerInfo.team == 1 ?
                gameState->team_a[playerInfo.id].position :
                gameState->team_b[playerInfo.id].position;
            //_startState = startState;
            _goalState = gameState->ball.position;
            //_goalState = goalState;

            _nodes.emplace_back(_startState, nullptr);

            setStepSize(g_settings.robot_radius * 2);
            setMaxIterations(1000);
            setGoalBias(0.5);
            setGoalMaxDistance(g_settings.robot_radius / 2 + g_settings.ball_radius);
            setObstacleDistance(g_settings.robot_radius * 2);
            std::cout << "1st x:" << _nodes.back().state().x << " y:" << _nodes.back().state().y << "\n";
        };

        rfsim_vec2& goalState() { return _goalState; };
        void setGoalState(rfsim_vec2& goalState) { _goalState = goalState; };

        double stepSize() { return _stepSize; };
        void setStepSize(double stepSize) { _stepSize = stepSize; };

        int maxIterations() { return _maxIterations; };
        void setMaxIterations(int maxIterations) { _maxIterations = maxIterations; };

        double goalBias() { return _goalBias; };
        void setGoalBias(double goalBias) { _goalBias = goalBias; };

        double goalMaxDistance() { return _goalMaxDistance; };
        void setGoalMaxDistance(double goalMaxDistance) { _goalMaxDistance = goalMaxDistance; };

        double obstacleDistance() { return _obstacleDistance; };
        void setObstacleDistance(double obstacleDistance) { _obstacleDistance = obstacleDistance; };

        Node* grow() {
            double r = (double) rand() / RAND_MAX;
            if (r < goalBias()) {
                //std::cout << "to goal!    ";
                return extend(goalState());
            } else {
                // std::cout << "bruh...     ";
                rfsim_vec2 ts = randomState();
                // std::cout << "!!!x:" << ts.x << " y:" << ts.y << "!!!";
                return extend(ts);
            }
        };

        Node* extend(const rfsim_vec2& target) {
            Node* source = nearest(target);

            rfsim_vec2 delta = {
                target.x - source->state().x,
                target.y - source->state().y
            };

            float dlength = (float) getLength(delta);

            delta.x /= dlength;
            delta.y /= dlength;

            delta.x *= stepSize();
            delta.y *= stepSize();

            rfsim_vec2 nextState = {
                source->state().x + delta.x,
                source->state().y + delta.y,
            };

            //найти расстояние до каждого прочего игрока
            double minDist = std::numeric_limits<double>::max();

            for (int i = 0; i < g_start_info.team_size; i++) {
                if (_playerInfo.team != 1 || (_playerInfo.team == 1 && _playerInfo.id != i)) {
                    double tmp = getDistance(nextState, _gameState->team_a[i].position);
                    if (tmp < minDist) minDist = tmp;
                }
            }

            for (int i = 0; i < g_start_info.team_size; i++) {
                if (_playerInfo.team == 1 || (_playerInfo.team != 1 && _playerInfo.id != i)) {
                    double tmp = getDistance(nextState, _gameState->team_b[i].position);
                    if (tmp < minDist) minDist = tmp;
                }
            }

            if (!(minDist > obstacleDistance())) {
                return nullptr;
            }

            _nodes.emplace_back(nextState, source);
            return &_nodes.back();
        }

        Node* nearest(const rfsim_vec2& target) {
            double minDist = std::numeric_limits<double>::max();
            Node* res = nullptr;

            for(auto& node : _nodes) {
                double tmp = getDistance(node.state(), target);
                if (tmp < minDist) {
                    minDist = tmp;
                    res = &node;
                }
            }

            return res;
        }

        bool run() {
            // std::cout << stepSize() << "\n";
            // std::cout << goalMaxDistance() << "\n";
            // std::cout << "x:" << _goalState.x << " y:" << _goalState.y << "\n";
            for (int i = 0; i < _maxIterations; i++) {
                Node* newNode = grow();

                if (newNode && getDistance(newNode->state(), _goalState) < goalMaxDistance())
                    return true;
            }

            for (auto& node: _nodes) {

            }

            return false;
        }

        std::deque<rfsim_vec2> getPath() {
            std::deque<rfsim_vec2> res;
            auto& tmp = _nodes.back();
            do
            {
                res.push_front(tmp.state());
                tmp = *(tmp.parent());
            } while (tmp.parent() != nullptr);

            return res;
        }


        rfsim_vec2 randomState() {
            return {
                (float) rand() / RAND_MAX * g_settings.field_Size.x,
                (float) rand() / RAND_MAX * g_settings.field_Size.y
            };
        }



    private:
        rfsim_game_state_info* _gameState;

        PlayerInfo _playerInfo;

        rfsim_vec2 _startState;
        rfsim_vec2 _goalState;

        std::deque<Node> _nodes;
        double _stepSize;
        int _maxIterations;
        double _goalBias;
        double _goalMaxDistance;
        double _obstacleDistance;
};

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

    /** DWA dwa_velocity cost weight */
    float velocity_cost_weight;

} rrt_config;

static rrt_config RRT_CONFIG = {
        .max_speed             = 1.0f,// is set later from simulator settings
        .min_speed             = 0.0f,
        .max_yaw_speed         = (float) (360.0 * M_PI / 180.0),
        .max_lin_accel         = 0.3f,
        .max_yaw_accel         = (float) (360.0 * M_PI / 180.0),
        .lin_speed_resolution  = 0.005f,// must be <= max_lin_accel * dt
        .yaw_speed_resolution  = (float) (2.0 * M_PI / 180.0),
        .dt                    = 0.1f,
		.predict_time          = 2.0f,
		.heading_cost_weight   = 0.8f,
        .velocity_cost_weight  = 0.1f
};


typedef struct {
    float linear;
    float angular;
} rrt_velocity;

typedef struct {
    int    possible_v_size;
    float* possible_v;

    int    possible_w_size;
    float* possible_w;
} rrt_dynamic_window;

float norm(float x, float y) {
    return sqrtf(x * x + y * y);
}

float dist(rfsim_vec2 p1, rfsim_vec2 p2) {
    const float dx = p1.x - p2.x;
    const float dy = p1.y - p2.y;
    return sqrtf(dx * dx + dy * dy);
}

int signf(float x) {
    return (x > 0) - (x < 0);
}

float uniform_rand(float min, float max) {
    double scale = rand() / (double) RAND_MAX;
    return (float) (min + scale * (max - min));
}

void set_random_control(rfsim_control* control, bool to_left) {
    control->left_wheel_velocity  = uniform_rand(0.0f, 0.5f) * (to_left ? 1.5f : 1.0f);
    control->right_wheel_velocity = uniform_rand(0.0f, 0.5f) * (to_left ? 1.0f : 1.5f);
}

float clampf(float v, float min, float max) {
    return fmaxf(fminf(v, max), min);
}

float safe_divf(float dividend, float divisor) {
    return fabsf(dividend) < EPS ? 0.0f : dividend / divisor;
}




rfsim_control velocity_rrt_to_rfsim(rrt_velocity v, float dt) {
    const float l = g_settings.robot_wheel_x_offset * 2;

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

rrt_velocity velocity_rfsim_to_rrt(rfsim_control v, float dt) {
    const float v_l = v.left_wheel_velocity;
    const float v_r = v.right_wheel_velocity;

    const float l = g_settings.robot_wheel_x_offset * 2;

    rrt_velocity result;
    if (fabsf(v_l - v_r) < EPS) {
        result = (rrt_velocity){
                .linear  = (v_r + v_l) / 2.0f,
                .angular = 0.0f,
        };
    } else {
        const float R = l / 2.0f * (v_l + v_r) / (v_l - v_r);
        const float w = (v_l - v_r) / l;

        const float x_n = sinf(w * dt) * R;
        const float y_n = cosf(w * dt) * (-R) + R;

        result = (rrt_velocity){
                .linear  = clampf(norm(x_n, y_n) / dt, RRT_CONFIG.min_speed, RRT_CONFIG.max_speed),
                .angular = clampf(w, -RRT_CONFIG.max_yaw_speed, RRT_CONFIG.max_yaw_speed),
        };
    }

    return result;
}

void free_dynamic_window(rrt_dynamic_window* window) {
    free(window->possible_v);
    free(window->possible_w);
    free(window);
}

rrt_dynamic_window* new_dynamic_window(rrt_velocity velocity, const rrt_config* config) {

    const float min_v = fmaxf(config->min_speed, velocity.linear - config->max_lin_accel * config->dt);
    const float max_v = fminf(config->max_speed, velocity.linear + config->max_lin_accel * config->dt);
    const float min_w = fmaxf(-config->max_yaw_speed, velocity.angular - config->max_yaw_accel * config->dt);
    const float max_w = fminf(config->max_yaw_speed, velocity.angular + config->max_yaw_accel * config->dt);

    const int possible_v_size = (int) floorf(safe_divf(max_v - min_v, config->lin_speed_resolution));
    const int possible_w_size = (int) floorf(safe_divf(max_w - min_w, config->yaw_speed_resolution));

    rrt_dynamic_window* window = (rrt_dynamic_window*)malloc(sizeof(rrt_dynamic_window));

    window->possible_v_size = possible_v_size;
    window->possible_w_size = possible_w_size;
    window->possible_v      = (float*)malloc(possible_v_size * sizeof(*FIELD_OF(rrt_dynamic_window, possible_v)));
    window->possible_w      = (float*)malloc(possible_w_size * sizeof(*FIELD_OF(rrt_dynamic_window, possible_w)));

    for (int i = 0; i < possible_v_size; ++i) {
        window->possible_v[i] = min_v + (float) i * config->lin_speed_resolution;
    }

    for (int i = 0; i < possible_w_size; ++i) {
        window->possible_w[i] = min_w + (float) i * config->yaw_speed_resolution;
    }

    return window;
}

rfsim_body move_body(rfsim_body body, rrt_velocity velocity, float dt) {

    const float new_angle = body.angle + velocity.angular * dt;
	rfsim_body new_body;
	new_body.angle = new_angle;
	new_body.position.x = body.position.x + velocity.linear * cosf(new_angle) * dt;
	new_body.position.y = body.position.y + velocity.linear * sinf(new_angle) * dt;
    return new_body;
}

float calc_velocity_cost(rrt_velocity source, rrt_velocity target, const rrt_config* config) {

    return 1 - target.linear / config->max_speed;
}

float calc_heading_cost(rfsim_body source, rfsim_vec2 target) {
    return dist(source.position, target) / FIELD_DIAG;
}

rrt_velocity plan(
        rfsim_body             source,
        rrt_velocity           source_velocity,
        rfsim_vec2             target,
        const rrt_config*      config) {

    rrt_dynamic_window* window = new_dynamic_window(source_velocity, config);
	float        best_cost           = FLT_MAX;
    float        best_velocity_cost  = FLT_MAX;
    float        best_heading_cost   = FLT_MAX;
    float        best_clearance_cost = FLT_MAX;
    rrt_velocity best_velocity       = { 0, 0 };
	rrt_velocity target_velocity;
    for (int i = 0; i < window->possible_v_size; ++i) {
        for (int j = 0; j < window->possible_w_size; ++j) {
            target_velocity = {
                    .linear  = window->possible_v[i],
                    .angular = window->possible_w[j],
            };

            const rfsim_body target_pose = move_body(source, target_velocity, config->predict_time);
			const float velocity_cost  = calc_velocity_cost(source_velocity, target_velocity, config);
            const float heading_cost   = calc_heading_cost(target_pose, target);


            const float target_cost = config->velocity_cost_weight * velocity_cost +
                                      config->heading_cost_weight * heading_cost;

            if (target_cost < best_cost) {
                best_cost           = target_cost;
                best_velocity_cost  = velocity_cost;
                best_heading_cost   = heading_cost;
                best_velocity       = target_velocity;
            }
			}
        }

	free_dynamic_window(window);
    return best_velocity;
}

rfsim_control rrt_move_to_target(
        rfsim_control          initial_control,
        rfsim_body             initial_position,
        const float            dt,
        rfsim_vec2             target) {

    rrt_velocity  initial_v = velocity_rfsim_to_rrt(initial_control, dt);
    rrt_velocity  result_v  = plan(initial_position, initial_v, target, &RRT_CONFIG);
    rfsim_control converted = velocity_rrt_to_rfsim(result_v, dt);

    return converted;
}


RFSIM_DEFINE_FUNCTION_INIT {
    std::strcpy(context->name, "RRT movement");
    std::strcpy(context->description, "Moving to ball using rrt path finding algorithm");
    return rfsim_status_success;

};

RFSIM_DEFINE_FUNCTION_BEGIN_GAME {
    g_settings = *settings;
    g_start_info = *start;
    // std::cout << "rr:" << g_settings.robot_radius << "\n";
    // std::cout << "fx:" << g_settings.field_Size.x << " fy:" << g_settings.field_Size.y << "\n";
	RRT_CONFIG.max_speed = settings->robot_max_speed;
	FIELD_DIAG = norm(settings->field_Size.x, settings->field_Size.y);
    return rfsim_status_success;
};

float x_ar[40];
float y_ar[40];
int posi = 1;

RFSIM_DEFINE_FUNCTION_TICK_GAME {
	if (flag) {
        Tree rrt = Tree(state, { 1, 0 });
        // std::cout << "rr:" << g_settings.robot_radius << "\n";
        // std::cout << "br:" << g_settings.ball_radius << "\n";

        // std::cout << "fx:" << g_settings.field_Size.x << " fy:" << g_settings.field_Size.y << "\n";

        // std::cout << "pl x:" << state->team_a[0].position.x << " y:" << state->team_a[0].position.y << "\n";
        // std::cout << "bl x:" << state->ball.position.x << " y:" << state->ball.position.y << "\n";

        if (rrt.run()) {
            std::cout << "done\n";
            std::deque<rfsim_vec2> path = rrt.getPath();
            int i = 0;
            for (auto& pos: path) {
                x_ar[i] = pos.x;
				y_ar[i] = pos.y;

				i++;
			}
        } else {
				printf("RRT ERROR\n");
			}


	}

	flag = false;
	const rfsim_body* forward     = &state->team_a[0];
    rfsim_control*    forward_ctl = &state->team_a_control[0];
	rfsim_vec2 target;
	target.x = x_ar[posi];
	target.y = y_ar[posi];
	if (dist(forward->position, target) > 0.1) {
		*forward_ctl = rrt_move_to_target(*forward_ctl,*forward, state->dt, target);
	} else {
		posi = posi + 1;
		target.x = x_ar[posi];
		target.y = y_ar[posi];
		*forward_ctl = rrt_move_to_target(*forward_ctl,*forward, state->dt, target);
	}
    return rfsim_status_success;
};

RFSIM_DEFINE_FUNCTION_END_GAME {
    return rfsim_status_success;
};

RFSIM_DEFINE_FUNCTION_FINALIZE {
    return rfsim_status_success;
};
