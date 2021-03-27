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

#ifndef RFSIM_TESTSCENARIO_HPP
#define RFSIM_TESTSCENARIO_HPP

#include <logic/GameScenario.hpp>

namespace rfsim {

    class TestScenario: public GameScenario {
    public:
        ~TestScenario() override = default;

        std::string GetName() const override {
            return "Test Scenario";
        }

        std::string GetDescription() const override {
            return "Basic scenario used for test purposes only";
        }

        std::shared_ptr<Game> CreateGame() const override {
            auto game = std::make_shared<Game>();

            // Some constants
            const auto pi = glm::pi<float>();
            const float fieldLength = 16;
            const float fieldWidth = 9;

            // Team size (total x2 robots)
            game->teamSize = 6;

            // todo: change properties to real ones
            auto& physicsProperties = game->physicsGameProperties;
            physicsProperties.fieldFriction = 0.5f;
            physicsProperties.robotRadius = 0.2f;
            physicsProperties.robotHeight = 0.1f;
            physicsProperties.robotMass = 1.0f;
            physicsProperties.robotFriction = 0.25f;
            physicsProperties.robotRestitution = 0.1f;
            physicsProperties.robotLeftMotorOffset = { 0, -0.8f };
            physicsProperties.robotRightMotorOffset = { 0, 0.8f };
            physicsProperties.ballRadius = 0.1f;
            physicsProperties.ballMass = 0.05f;
            physicsProperties.ballFriction = 0.005f;
            physicsProperties.ballRestitution = 0.05f;

            // Field settings will be fixed (but ball placement can differ)
            auto& beginInfo = game->physicsGameInitInfo;
            beginInfo.fieldTopLeftBounds     = { 0.5f, 0.5f };
            beginInfo.fieldBottomRightBounds = { fieldLength - 0.5f,  fieldWidth - 0.5f };
            beginInfo.roomTopLeftBounds      = { 0, 0 };
            beginInfo.roomBottomRightBounds  = { fieldLength, fieldWidth };
            beginInfo.ballPosition = { fieldLength * 0.5f, fieldWidth * 0.5f };

            // Initial robots placement
            for (int i = 0; i < game->teamSize; i++) {
                beginInfo.robotsTeamA.push_back({ i,     { fieldLength * 0.25f, fieldWidth * 0.5f + fieldWidth * 0.3f * ((i - 2.5f) / 2.5f) }, 0 });
                game->robotMotorPowerA.emplace_back(0,0);
                beginInfo.robotsTeamB.push_back({ (int)(i + game->teamSize), { fieldLength * 0.75f, fieldWidth * 0.5f + fieldWidth * 0.3f * ((i - 2.5f) / 2.5f) }, pi });
                game->robotMotorPowerB.emplace_back(0,0);
            }

            // Graphics is exact copy of ph settings + init info
            auto& sceneSettings = game->graphicsSceneSettings;
            sceneSettings.ballRadius = physicsProperties.ballRadius;
            sceneSettings.ballPosition = beginInfo.ballPosition;
            sceneSettings.robotRadius = physicsProperties.robotRadius;
            sceneSettings.fieldTopLeftBounds = beginInfo.fieldTopLeftBounds;
            sceneSettings.fieldBottomRightBounds = beginInfo.fieldBottomRightBounds;
            sceneSettings.roomTopLeftBounds = beginInfo.roomTopLeftBounds;
            sceneSettings.roomBottomRightBounds = beginInfo.roomBottomRightBounds;
            sceneSettings.robotsTeamA = beginInfo.robotsTeamA;
            sceneSettings.robotsTeamB = beginInfo.robotsTeamB;

            return game;
        }
    };

}

#endif //RFSIM_TESTSCENARIO_HPP
