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

#ifndef RFSIM_SCENARIOCOMMON_HPP
#define RFSIM_SCENARIOCOMMON_HPP

#include <physics/PhysicsGameProperties.hpp>
#include <physics/PhysicsGameInitInfo.hpp>
#include <graphics/GraphicsSceneSettings.hpp>
#include <glm/vec2.hpp>

namespace rfsim {

    class ScenarioCommon {
    public:

        static const unsigned int DEFAULT_TEAM_SIZE = 6;

        static glm::vec2 GetDefaultFieldSize() {
            return {9.0f, 6.0f};
        }

        static glm::vec2 GetDefaultFieldBorderOffset() {
            return { 0.7f, 0.7f };
        }

        static PhysicsGameProperties GetDefaultPhysicsProperties() {
            PhysicsGameProperties pp;
            pp.fieldFriction = 0.0f;
            pp.robotRadius = 0.15f;
            pp.robotHeight = 0.1f;
            pp.robotMass = 1.0f;
            pp.robotFriction = 0.25f;
            pp.robotRestitution = 0.1f;
            pp.robotMaxSpeed = 100.0f;
            pp.robotWheelXOffset = 0.15f;
            pp.ballRadius = 0.08f;
            pp.ballMass = 0.05f;
            pp.ballFriction = 0.005f;
            pp.ballRestitution = 0.05f;

            return pp;
        }

        static GraphicsSceneSettings GetDefaultSceneSettingsFromPhysics(const PhysicsGameProperties& pp, const PhysicsGameInitInfo& bi) {
            GraphicsSceneSettings ss;
            ss.ballRadius = pp.ballRadius;
            ss.ballPosition = bi.ballPosition;
            ss.robotRadius = pp.robotRadius;
            ss.fieldTopLeftBounds = bi.fieldTopLeftBounds;
            ss.fieldBottomRightBounds = bi.fieldBottomRightBounds;
            ss.roomTopLeftBounds = bi.roomTopLeftBounds;
            ss.roomBottomRightBounds = bi.roomBottomRightBounds;
            ss.robotsTeamA = bi.robotsTeamA;
            ss.robotsTeamB = bi.robotsTeamB;

            return ss;
        }

    };

}

#endif //RFSIM_SCENARIOCOMMON_HPP
