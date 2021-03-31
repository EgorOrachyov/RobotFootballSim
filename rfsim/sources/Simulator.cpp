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
#include <logic/GameManager.hpp>
#include <logic/GameRulesManager.hpp>
#include <utils/ConfigManager.hpp>

#include <scenario/Scrum.hpp>
#include <scenario/Duel.hpp>

#include <rules/BallHit.hpp>
#include <rules/NoCollisions.hpp>
#include <rules/TimeLimit.hpp>

namespace rfsim {

    Simulator::Simulator(int argc, const char *const *argv) {
        // Parse user args
        for (int i = 0; i < argc; i++) {
            mArgs.emplace_back(argv[i]);
        }

        mConfigManager = std::make_shared<ConfigManager>("config.json");
        mWindowWidth = mConfigManager->GetWindowSize().x;
        mWindowHeight = mConfigManager->GetWindowSize().y;

        // Setup main window and window manager
        mWindowManager = std::make_shared<WindowManager>();
        mPrimaryWindow = mWindowManager->CreateNewWindow({mWindowWidth, mWindowHeight}, "Robot Football Sim");
        mPainter = std::make_shared<Painter>(glm::ivec4{0, 0, mWindowWidth, mWindowHeight}, glm::vec4{0, 0, 1.0f, 1.0f}, mPrimaryWindow);
        mGraphicsServer = std::make_shared<GraphicsServer>(mPrimaryWindow, mPainter, mConfigManager->GetResourcesPath());
        mPhysicsServer = std::make_shared<PhysicsServer>();
        mGameManager = std::make_shared<GameManager>();
        mGameRulesManager = std::make_shared<GameRulesManager>();
        mAlgorithmManager = std::make_shared<AlgorithmManager>(mConfigManager->GetPluginPathPrefix());

        // This is default algorithm and scenario (does not change order)
        for (const auto &a : mConfigManager->GetPluginsPaths()) {
            mAlgorithmManager->Load(a);
        }

        mGameManager->AddScenario(std::make_shared<Scrum>());
        mGameManager->AddScenario(std::make_shared<Duel>());

        mGameRulesManager->AddRule(std::make_shared<TimeLimit>(20.0f));
        mGameRulesManager->AddRule(std::make_shared<TimeLimit>(40.0f));
        mGameRulesManager->AddRule(std::make_shared<TimeLimit>(60.0f));
        mGameRulesManager->AddRule(std::make_shared<BallHit>());
        mGameRulesManager->AddRule(std::make_shared<NoCollisions>());
    }

    Simulator::~Simulator() {
        // Release in reverse order
        mGameRulesManager = nullptr;
        mGameManager = nullptr;
        mAlgorithmManager = nullptr;
        mPhysicsServer = nullptr;
        mGraphicsServer = nullptr;
        mPainter = nullptr;
        mPrimaryWindow = nullptr;
        mWindowManager = nullptr;
        mConfigManager = nullptr;
    }

    int Simulator::Run() {
        auto algo = mAlgorithmManager->GetAlgorithmAt(0);
        auto game = mGameManager->CreateGame(0);

        mPhysicsServer->SetGameProperties(game->physicsGameProperties);
        mPhysicsServer->BeginGame(game->physicsGameInitInfo);
        mGraphicsServer->BeginGame(game->graphicsSceneSettings);

        algo->BeginGame(*game);

        float dt = 1.0f / 60.0f;
        float t = 0.0f;
        uint64_t frameCount = 0;

        while (!mPrimaryWindow->ShouldClose()) {
            // Update loop strategy:
            // 1) Update physics - simulate game with step (step is prev frame delta)
            // 2) Draw game state and gui
            // 3) Tick algorithm control (if required)
            // 4) Update physics settings (motors power) (if required)

            // Return true if must continue physics sim step
            auto onFixedStep = [&] (float fixedDt) {
                algo->TickGame(fixedDt, t, *game);
                t += fixedDt;

                return true;
            };

            mPhysicsServer->FrameStep(game, onFixedStep, dt);
            mPhysicsServer->GetCurrentGameState(game->physicsGameState);

            mGraphicsServer->BeginDraw(dt, game->physicsGameState);
            mGraphicsServer->DrawStaticObjects();
            mGraphicsServer->DrawDynamicObjects();
            mGraphicsServer->DrawAuxInfo();
            mGraphicsServer->DrawPostUI();
            mGraphicsServer->EndDraw();

            mWindowManager->UpdateEvents();
            mWindowManager->SwapBuffers();
            mPainter->FitToFramebufferArea();

            frameCount++;
            t += dt;
        }

        algo->EndGame(*game);
        mGraphicsServer->EndGame();
        mPhysicsServer->EndGame();

        return 0;
    }

}
