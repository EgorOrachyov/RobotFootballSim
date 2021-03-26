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

#ifndef RFSIM_PHYSICSGAMESTATE_HPP
#define RFSIM_PHYSICSGAMESTATE_HPP

#include <logic/GameCommon.hpp>
#include <vector>

namespace rfsim {

    /**
     * @brief Game state.
     *
     * Current state of the game.
     * Includes: robots positions, orientation,
     * ball position, information about collisions and etc.
     */
    struct PhysicsGameState {
        // Index of a robot in a vector is its ID
        std::vector<BodyState> robots;
        BodyState ball;

        std::vector<RobotCollisionInfo> robotRobotCollisions;
        std::vector<int> robotFieldBoundsCollisions;
        std::vector<int> robotRoomBoundsCollisions;
        std::vector<int> robotBallCollisions;

        bool isBallInsideGoal;
        bool isBallOutOfFieldBounds;
        bool isBallRobotCollision;
    };

}

#endif //RFSIM_PHYSICSGAMESTATE_HPP