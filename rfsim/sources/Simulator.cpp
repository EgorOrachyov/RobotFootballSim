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

#include <Simulator.hpp>

#include <cmath>
#include <glm/gtc/constants.hpp>
#include <graphics/PainterEngine.hpp>
#include <graphics/WindowManager.hpp>
#include <graphics/GraphicsServer.hpp>
#include <physics/PhysicsServer.hpp>
#include <AlgorithmManager.hpp>

namespace rfsim {

    Simulator::Simulator(int argc, const char *const *argv) {
        // Parse user args
        for (int i = 0; i < argc; i++) {
            mArgs.emplace_back(argv[i]);
        }

        // Setup main window and window manager
        mWindowManager = std::make_shared<WindowManager>();
        mPrimaryWindow = mWindowManager->CreateWindow({1280, 720}, "Robot Football Sim");
        mPainter = std::make_shared<PainterEngine>(glm::ivec4{0, 0, 1280, 720}, glm::vec4{0, 0, 1280, 720}, mPrimaryWindow);
        mGraphicsServer = std::make_shared<GraphicsServer>(mPrimaryWindow, mPainter, mResourcesPath);
        mPhysicsServer = std::make_shared<PhysicsServer>();
        mAlgorithmManager = std::make_shared<AlgorithmManager>(mPluginsPath);
    }

    Simulator::~Simulator() {
        // Release in reverse order
        mAlgorithmManager = nullptr;
        mPhysicsServer = nullptr;
        mGraphicsServer = nullptr;
        mPainter = nullptr;
        mPrimaryWindow = nullptr;
        mWindowManager = nullptr;
    }

    int Simulator::Run() {
        const auto pi = glm::pi<float>();

        const float fieldLength = 16;
        const float fieldWidth = 9;

        auto algo = mAlgorithmManager->Load("randommove");

        // todo: change properties to real ones
        PhysicsGameProperties physicsProperties = {};
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

        mPhysicsServer->SetGameProperties(physicsProperties);

        PhysicsGameInitInfo beginInfo = {};
        beginInfo.fieldTopLeftBounds     = { 0.5f, 0.5f };
        beginInfo.fieldBottomRightBounds = { fieldLength - 0.5f,  fieldWidth - 0.5f };
        beginInfo.roomTopLeftBounds      = { 0, 0 };
        beginInfo.roomBottomRightBounds  = { fieldLength, fieldWidth };
        beginInfo.ballPosition = { fieldLength * 0.5f, fieldWidth * 0.5f };
        for (int i = 0; i < 6; i++) {
            beginInfo.robotsTeamA.push_back({ i,     { fieldLength * 0.25f, fieldWidth * 0.5f + fieldWidth * 0.3f * ((i - 2.5f) / 2.5f) }, 0 });
            beginInfo.robotsTeamB.push_back({ i + 6, { fieldLength * 0.75f, fieldWidth * 0.5f + fieldWidth * 0.3f * ((i - 2.5f) / 2.5f) }, pi });
        }

        GraphicsSceneSettings sceneSettings;
        sceneSettings.ballRadius = physicsProperties.ballRadius;
        sceneSettings.ballPosition = beginInfo.ballPosition;
        sceneSettings.robotRadius = physicsProperties.robotRadius;
        sceneSettings.fieldTopLeftBounds = beginInfo.fieldTopLeftBounds;
        sceneSettings.fieldBottomRightBounds = beginInfo.fieldBottomRightBounds;
        sceneSettings.roomTopLeftBounds = beginInfo.roomTopLeftBounds;
        sceneSettings.roomBottomRightBounds = beginInfo.roomBottomRightBounds;
        sceneSettings.robotsTeamA = beginInfo.robotsTeamA;
        sceneSettings.robotsTeamB = beginInfo.robotsTeamB;

        mPhysicsServer->BeginGame(beginInfo);
        mGraphicsServer->BeginGame(sceneSettings);

        float dt = 1.0f / 60.0f;
        uint64_t frameCount = 0;

        while (!mPrimaryWindow->ShouldClose()) {

            // todo: remove (it is algo specific logic)
            for (int i = 0; i < beginInfo.robotsTeamA.size() * 2; i++) {
                mPhysicsServer->UpdateMotorsPower(i, 40 * (float)rand() / RAND_MAX, 70 * (float)rand() / RAND_MAX);
            }

            PhysicsGameState state;

            mPhysicsServer->GameStep(dt);
            mPhysicsServer->GetCurrentGameState(state);

            mGraphicsServer->BeginDraw(state);
            mGraphicsServer->DrawStaticObjects();
            mGraphicsServer->DrawDynamicObjects();
            mGraphicsServer->DrawAuxInfo();
            mGraphicsServer->DrawPostUI();
            mGraphicsServer->EndDraw();

            mWindowManager->Update();
            mPainter->FitToFramebufferArea();

            frameCount++;
        }

        mGraphicsServer->EndGame();
        mPhysicsServer->EndGame();

        return 0;
    }

}
