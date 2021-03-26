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

#ifndef RFSIM_GRAPHICSSCENESETTINGS_HPP
#define RFSIM_GRAPHICSSCENESETTINGS_HPP

#include <GameCommon.hpp>
#include <glm/vec2.hpp>
#include <vector>

namespace rfsim {

    struct GraphicsSceneSettings {
        float robotRadius = 0.0f;
        float ballRadius = 0.0f;

        glm::vec2 ballPosition;

        std::vector<RobotInitInfo> robotsTeamA;
        std::vector<RobotInitInfo> robotsTeamB;

        // Hard wall bounds.
        glm::vec2 roomTopLeftBounds;
        glm::vec2 roomBottomRightBounds;

        // Field bounds, they won't be used for collision,
        // but only for determining if a ball is out of bounds.
        glm::vec2 fieldTopLeftBounds;
        glm::vec2 fieldBottomRightBounds;
    };

}

#endif //RFSIM_GRAPHICSSCENESETTINGS_HPP