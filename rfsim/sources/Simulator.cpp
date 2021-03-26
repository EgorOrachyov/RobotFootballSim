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

#include <glm/gtc/constants.hpp>
#include <graphics/Painter.hpp>
#include <graphics/WindowManager.hpp>
#include <graphics/GraphicsServer.hpp>
#include <physics/PhysicsServer.hpp>
#include <logic/AlgorithmManager.hpp>
#include <logic/Game.hpp>
#include <gui/GuiApplication.hpp>

namespace rfsim {

    Simulator::Simulator(int argc, const char *const *argv) {
        // Parse user args
        for (int i = 0; i < argc; i++) {
            mArgs.emplace_back(argv[i]);
        }

        // Setup main window and window manager
        mWindowManager = std::make_shared<WindowManager>();
        mPrimaryWindow = mWindowManager->CreateWindow({1280, 720}, "Robot Football Sim");
        mPainter = std::make_shared<Painter>(glm::ivec4{0, 0, 1280, 720}, glm::vec4{0, 0, 1280, 720}, mPrimaryWindow);
        mGraphicsServer = std::make_shared<GraphicsServer>(mPrimaryWindow, mPainter, mResourcesPath);
        mPhysicsServer = std::make_shared<PhysicsServer>();
        mAlgorithmManager = std::make_shared<AlgorithmManager>(mPluginsPath);
        mApplication = std::make_shared<GuiApplication>(mPrimaryWindow, mWindowManager, mPainter, mPhysicsServer, mGraphicsServer, mAlgorithmManager, mResourcesPath);
    }

    Simulator::~Simulator() {
        // Release in reverse order
        mApplication = nullptr;
        mAlgorithmManager = nullptr;
        mPhysicsServer = nullptr;
        mGraphicsServer = nullptr;
        mPainter = nullptr;
        mPrimaryWindow = nullptr;
        mWindowManager = nullptr;
    }

    int Simulator::Run() {
        return mApplication->Run();

        const auto pi = glm::pi<float>();

        const float fieldLength = 16;
        const float fieldWidth = 9;

        auto algo = mAlgorithmManager->Load("randommove");
        auto game = std::make_shared<Game>();

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

        mPhysicsServer->SetGameProperties(physicsProperties);

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
        algo->BeginGame(*game);

        float dt = 1.0f / 60.0f;
        uint64_t frameCount = 0;

        while (!mPrimaryWindow->ShouldClose()) {
            // Update loop strategy:
            // 1) Update physics - simulate game with step (step is prev frame delta)
            // 2) Draw game state and gui
            // 3) Tick algorithm control (if required)
            // 4) Update physics settings (motors power) (if required)

            PhysicsGameState state;
            mPhysicsServer->GameStep(dt);
            mPhysicsServer->GetCurrentGameState(state);

            mGraphicsServer->BeginDraw(state);
            mGraphicsServer->DrawStaticObjects();
            mGraphicsServer->DrawDynamicObjects();
            mGraphicsServer->DrawAuxInfo();
            mGraphicsServer->DrawPostUI();
            mGraphicsServer->EndDraw();

            mWindowManager->UpdateEvents();
            mWindowManager->SwapBuffers();
            mPainter->FitToFramebufferArea();

            game->physicsGameState = state;
            algo->TickGame(*game);

            for (int i = 0; i < game->teamSize; i++) {
                auto id = game->physicsGameInitInfo.robotsTeamA[i].id;
                auto power = game->robotMotorPowerA[i];
                mPhysicsServer->UpdateMotorsPower(id, power.x, power.y);
            }

            for (int i = 0; i < game->teamSize; i++) {
                auto id = game->physicsGameInitInfo.robotsTeamB[i].id;
                auto power = game->robotMotorPowerB[i];
                mPhysicsServer->UpdateMotorsPower(id, power.x, power.y);
            }

            frameCount++;
        }

        algo->EndGame(*game);
        mGraphicsServer->EndGame();
        mPhysicsServer->EndGame();

        return 0;
    }

}
