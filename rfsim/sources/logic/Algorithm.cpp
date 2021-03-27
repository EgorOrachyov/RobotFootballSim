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

#include <logic/Algorithm.hpp>
#include <iostream>

namespace rfsim {

    static rfsim_vec2 to_rfsim_vec2(const glm::vec2& v) {
        rfsim_vec2 r;
        r.x = v.x;
        r.y = v.y;
        return r;
    }

    struct rfsim_control to_rfsim_control(const glm::vec2& v) {
        rfsim_control r;
        r.left_motor_force = v.x;
        r.right_motor_force = v.y;
        return r;
    }

    static glm::vec2 to_vec2(const rfsim_control& c) {
        glm::vec2 r;
        r.x = c.left_motor_force;
        r.y = c.right_motor_force;
        return r;
    }

    Algorithm::~Algorithm() {
        if (initialized) {
            auto finalize = finalizeFunction;
            auto status = finalize(&algoState);

            if (status != rfsim_status_success) {
                std::cerr << "Failed to finalize algorithm context "
                          << algoState.name << std::endl;
            }
        }
    }
    
    bool Algorithm::Init(dynalo::library &library) {
        initFunction = library.get_function<rfsim_status(rfsim_algo_state*)>(RFSIM_FUNCTION_INIT_NAME);
        beginGameFunction = library.get_function<rfsim_status(rfsim_algo_state*, const rfsim_game_settings*, const rfsim_game_start_info*)>(RFSIM_FUNCTION_BEGIN_GAME_NAME);
        tickGameFunction = library.get_function<rfsim_status(rfsim_algo_state*, rfsim_game_state_info*)>(RFSIM_FUNCTION_TICK_GAME_NAME);
        endGameFunction = library.get_function<rfsim_status(rfsim_algo_state*)>(RFSIM_FUNCTION_END_GAME_NAME);
        finalizeFunction = library.get_function<rfsim_status(rfsim_algo_state*)>(RFSIM_FUNCTION_FINALIZE_NAME);

        auto init = initFunction;
        auto status = init(&algoState);

        return initialized = status == rfsim_status_success;
    }

    void Algorithm::GetAboutInfo(std::string &info) {
        info = algoState.name;
    }

    void Algorithm::BeginGame(Game& game) {
        auto& pII = game.physicsGameInitInfo;
        auto& pP = game.physicsGameProperties;

        rfsim_game_settings gameSettings{};
        gameSettings.ball_radius = pP.ballRadius;
        gameSettings.robot_radius = pP.robotRadius;
        gameSettings.field_Size = to_rfsim_vec2(pII.roomBottomRightBounds - pII.roomTopLeftBounds);

        rfsim_game_start_info startInfo{};
        startInfo.ball.position = to_rfsim_vec2(pII.ballPosition);
        startInfo.ball.angle = 0.0f;
        startInfo.team_size = (int) game.teamSize;

        for (int i = 0; i < game.teamSize; i++) {
            auto& r = pII.robotsTeamA[i];
            startInfo.team_a[i].angle = r.angle;
            startInfo.team_a[i].position = to_rfsim_vec2(r.position);
        }

        for (int i = 0; i < game.teamSize; i++) {
            auto& r = pII.robotsTeamB[i];
            startInfo.team_b[i].angle = r.angle;
            startInfo.team_b[i].position = to_rfsim_vec2(r.position);
        }

        assert(beginGameFunction(&algoState, &gameSettings, &startInfo) == rfsim_status_success);
    }

    void Algorithm::TickGame(float dt, float t, Game &game) {
        auto& pS = game.physicsGameState;
        auto& pII = game.physicsGameInitInfo;

        rfsim_game_state_info stateInfo;
        stateInfo.dt = dt;
        stateInfo.t = t;
        stateInfo.ball.position = to_rfsim_vec2(pS.ball.position);
        stateInfo.ball.angle = pS.ball.angle;

        // Set robot data and power for team A
        for (int i = 0; i < game.teamSize; i++) {
            auto id = pII.robotsTeamA[i].id;
            auto& r = pS.robots[id];

            stateInfo.team_a[i].angle = r.angle;
            stateInfo.team_a[i].position = to_rfsim_vec2(r.position);
            stateInfo.team_a_control[i] = to_rfsim_control(game.robotMotorPowerA[i]);
        }

        // Set robot data and power for team B
        for (int i = 0; i < game.teamSize; i++) {
            auto id = pII.robotsTeamB[i].id;
            auto& r = pS.robots[id];

            stateInfo.team_b[i].angle = r.angle;
            stateInfo.team_b[i].position = to_rfsim_vec2(r.position);
            stateInfo.team_b_control[i] = to_rfsim_control(game.robotMotorPowerB[i]);
        }

        assert(tickGameFunction(&algoState, &stateInfo) == rfsim_status_success);

        // Copy power info
        for (int i = 0; i < game.teamSize; i++) {
            game.robotMotorPowerA[i] = to_vec2(stateInfo.team_a_control[i]);
            game.robotMotorPowerB[i] = to_vec2(stateInfo.team_b_control[i]);
        }
    }

    void Algorithm::EndGame(Game& game) {
        assert(endGameFunction(&algoState) == rfsim_status_success);
    }

}