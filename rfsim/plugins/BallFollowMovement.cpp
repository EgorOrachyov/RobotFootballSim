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
#include <glm/glm.hpp>
#include <glm/vec2.hpp>
#include <iostream>
#include <random>
#include <chrono>
#include <cstring>
#include <cmath>

static int teamSize = 0;

RFSIM_DEFINE_FUNCTION_INIT {
    std::strcpy(context->name, "Ball Follow Movement (testing)");
    std::strcpy(context->description, "Movement to follow ball and hit it (does not avoids collisions)");
    return rfsim_status_success;
};

RFSIM_DEFINE_FUNCTION_BEGIN_GAME {
    teamSize = start->team_size;
    return rfsim_status_success;
};

RFSIM_DEFINE_FUNCTION_TICK_GAME {
    std::default_random_engine engine(std::chrono::system_clock::now().time_since_epoch().count());
    auto dist = std::uniform_real_distribution<float>(0.0, 1.0);
    auto basePowerA = 1.0f;
    auto basePowerB = 1.1f;

    auto& b = state->ball;
    auto bp = glm::vec2(b.position.x, b.position.y);

    for (int i = 0; i < teamSize; i++) {
        auto &r = state->team_a[i];
        auto rp = glm::vec2(r.position.x, r.position.y);
        auto a = r.angle;
        auto rd = glm::vec2(std::cos(a), std::sin(a));
        auto bd = bp - rp;

        auto dotRdBd = glm::dot(rd, bd) / glm::length(rd) / glm::length(bd);
        auto d = glm::length(bd) + 1.0f;
        auto left = (d + dotRdBd) * dist(engine);
        auto right = (d) * dist(engine);

        state->team_a_control[i] = {left * basePowerA, right * basePowerA};
    }

    for (int i = 0; i < teamSize; i++) {
        auto &r = state->team_b[i];
        auto rp = glm::vec2(r.position.x, r.position.y);
        auto a = r.angle;
        auto rd = glm::vec2(std::cos(a), std::sin(a));
        auto bd = bp - rp;

        auto dotRdBd = glm::dot(rd, bd) / glm::length(rd) / glm::length(bd);
        auto d = glm::length(bd) + 1.0f;
        auto left = (d + dotRdBd) * dist(engine);
        auto right = (d) * dist(engine);

        state->team_b_control[i] = {left * basePowerB, right * basePowerB};
    }

    return rfsim_status_success;
};

RFSIM_DEFINE_FUNCTION_END_GAME {
    return rfsim_status_success;
};

RFSIM_DEFINE_FUNCTION_FINALIZE {
    return rfsim_status_success;
};
