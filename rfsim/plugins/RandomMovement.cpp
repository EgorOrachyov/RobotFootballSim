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
#include <iostream>
#include <random>
#include <chrono>
#include <cstring>

static int teamSize = 0;

RFSIM_DEFINE_FUNCTION_INIT {
    std::strcpy(context->name, "Random Movement (testing)");
    std::strcpy(context->description, "Random movement of the robots with nearly fixed direction");
    return rfsim_status_success;
};

RFSIM_DEFINE_FUNCTION_BEGIN_GAME {
    teamSize = start->team_size;
    return rfsim_status_success;
};

RFSIM_DEFINE_FUNCTION_TICK_GAME {
    std::default_random_engine engine(std::chrono::system_clock::now().time_since_epoch().count());
    auto dist = std::uniform_real_distribution<float>(0.0, 1.0);

    for (int i = 0; i < teamSize; i++) {
        state->team_a_control[i].left_motor_force = 70.0f * dist(engine);
        state->team_a_control[i].right_motor_force = 45.0f * dist(engine);
    }

    for (int i = 0; i < teamSize; i++) {
        state->team_b_control[i].left_motor_force = 50.0f * dist(engine);
        state->team_b_control[i].right_motor_force = 65.0f * dist(engine);
    }

    return rfsim_status_success;
};

RFSIM_DEFINE_FUNCTION_END_GAME {
    return rfsim_status_success;
};

RFSIM_DEFINE_FUNCTION_FINALIZE {
    return rfsim_status_success;
};
