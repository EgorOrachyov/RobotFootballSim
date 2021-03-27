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

#ifndef RFSIM_GAME_HPP
#define RFSIM_GAME_HPP

#include <physics/PhysicsGameState.hpp>
#include <physics/PhysicsGameInitInfo.hpp>
#include <physics/PhysicsGameProperties.hpp>
#include <graphics/GraphicsSceneSettings.hpp>

namespace rfsim {

    /**
     * @brief Game state
     * Game specific information.
     * This structure must be created for each new game.
     */
    struct Game {
        unsigned int teamSize = 0;
        unsigned int teamScoreA = 0;
        unsigned int teamScoreB = 0;

        PhysicsGameState physicsGameState;
        PhysicsGameInitInfo physicsGameInitInfo;
        PhysicsGameProperties physicsGameProperties;
        GraphicsSceneSettings graphicsSceneSettings;

        std::vector<glm::vec2> robotMotorPowerA;
        std::vector<glm::vec2> robotMotorPowerB;
    };

}

#endif //RFSIM_GAME_HPP
