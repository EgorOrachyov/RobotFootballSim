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

static rfsim_game_settings g_settings;
static rfsim_game_start_info g_start_info;

static bool flag = true;

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

    return rfsim_status_success;
};

RFSIM_DEFINE_FUNCTION_TICK_GAME {
    if (flag) {
        Tree rrt = Tree(state, { 1, 0 });
        // std::cout << "rr:" << g_settings.robot_radius << "\n";
        // std::cout << "br:" << g_settings.ball_radius << "\n";

        // std::cout << "fx:" << g_settings.field_Size.x << " fy:" << g_settings.field_Size.y << "\n";

        // std::cout << "pl x:" << state->team_a[0].position.x << " y:" << state->team_a[0].position.y << "\n";
        // std::cout << "bl x:" << state->ball.position.x << " y:" << state->ball.position.y << "\n";
        if (rrt.run())
        {
            std::cout << "done\n";
            std::deque<rfsim_vec2> path = rrt.getPath();
            int i = 0;
            for (auto& pos: path) {
                std::cout << i << " x:" << pos.x << " y:" << pos.y << "\n";
                i++;
            }
        }
        flag = false;
    }

    return rfsim_status_success;
};

RFSIM_DEFINE_FUNCTION_END_GAME {
    return rfsim_status_success;
};

RFSIM_DEFINE_FUNCTION_FINALIZE {
    return rfsim_status_success;
};