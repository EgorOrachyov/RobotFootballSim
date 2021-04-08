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

#include <gui/GuiSimulator.hpp>
#include <gui/GuiMenuBar.hpp>
#include <graphics/Image.hpp>
#include <graphics/WindowManager.hpp>
#include <graphics/Painter.hpp>
#include <graphics/GraphicsServer.hpp>
#include <physics/PhysicsServer.hpp>
#include <logic/AlgorithmManager.hpp>
#include <logic/GameManager.hpp>
#include <logic/GameRulesManager.hpp>
#include <rules/CombineAnd.hpp>
#include <utils/ConfigManager.hpp>
#include <glm/gtc/constants.hpp>
#include <imgui.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>
#include <memory>

namespace rfsim {

    struct Flag {
        bool v = false;
    };

    GuiSimulator::GuiSimulator(int argc, const char* const* argv) : Simulator(argc, argv) {
        const auto SEP = "/";
        const auto prefix = mConfigManager->GetResourcesPath() + SEP + "sprites" + SEP;

        mMainMenuLogo = Image::LoadFromFilePath(prefix + "main-menu-logo.png");
        mMainMenuBall = Image::LoadFromFilePath(prefix + "soccer-ball.png");
    }

    int GuiSimulator::Run() {
#if defined(__APPLE__)
        // GL 3.2 + GLSL 150
        const char* glsl_version = "#version 150";
#else
        // GL 3.0 + GLSL 130
        const char* glsl_version = "#version 130";
#endif

        // Setup imgui
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();

        auto& io = ImGui::GetIO();
        io.FontGlobalScale *= mConfigManager->GetFontScale();

        auto& style = ImGui::GetStyle();
        style.ScaleAllSizes(mConfigManager->GetGuiScale());

        ImGui::StyleColorsDark();
        ImGui_ImplGlfw_InitForOpenGL(mPrimaryWindow->GetNativeHnd(), true);
        ImGui_ImplOpenGL3_Init(glsl_version);

        // Global simulator state
        GuiMenuBar menuBar;
        GameState gameState;
        std::shared_ptr<Game> game;
        std::shared_ptr<Algorithm> algo;
        std::shared_ptr<CombineAnd> rule = std::make_shared<CombineAnd>();
        std::string ruleTextInfo;

        // This is main menu related data
        bool needRefresh = true;
        bool beginGame;
        bool exit = false;
        int selectedScenario = -1;
        int selectedAlgo = -1;
        std::vector<Flag> selectedRules;
        std::vector<std::string> scenarios;
        std::vector<const char*> scenariosRaw;
        std::vector<std::string> algorithms;
        std::vector<const char*> algorithmsRaw;
        std::vector<std::string> rules;

        // In-game
        float dt;
        float t = 0.0f;
        bool needRestart = false;

        while (!mPrimaryWindow->ShouldClose() && !exit) {
            mWindowManager->UpdateEvents();
            mGraphicsServer->SetSettings(menuBar.graphicsSettings);

            // Start the Dear ImGui frame
            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();

            // Update delta time
            dt = 1.0f / ImGui::GetIO().Framerate;
            dt = dt > (1.0f / 30.f)? 1.0f / 30.0f: dt;

            // Draw main menu
            menuBar.Update();
            exit |= menuBar.quit;

            if (mState == State::MainMenu) {
                // Reset state
                if (needRefresh) {
                    selectedScenario = -1;
                    selectedAlgo = -1;

                    algorithms.clear();
                    scenarios.clear();
                    rules.clear();

                    mAlgorithmManager->GetAlgorithmsInfo(algorithms);
                    mGameManager->GetScenarioInfo(scenarios);
                    mGameRulesManager->GetRulesInfo(rules);

                    scenariosRaw.clear();
                    scenariosRaw.reserve(scenarios.size());

                    for (auto& s: scenarios)
                        scenariosRaw.push_back(s.data());

                    algorithmsRaw.clear();
                    algorithmsRaw.reserve(algorithms.size());

                    for (auto& a: algorithms)
                        algorithmsRaw.push_back(a.data());

                    selectedRules.clear();
                    selectedRules.resize(rules.size(), {});

                    needRefresh = false;
                }

                // Draw animated background logo
                {
                    static const auto pi = glm::pi<float>();
                    static float angle = 0.0f;
                    static float step = 1.0f;

                    auto size = mPrimaryWindow->GetFramebufferSize();
                    auto aspect = (float) size.x / (float) size.y;

                    mPainter->Clear();
                    mPainter->SetDrawArea({0, 0, size});
                    mPainter->SetDrawSpace({0, 0, aspect, 1});
                    mPainter->SetClearColor({mStyle.clearColor.x, mStyle.clearColor.y, mStyle.clearColor.z, mStyle.clearColor.w});
                    mPainter->SetBrushColor(glm::vec4 {1.0f});
                    mPainter->SetTransparentColor(glm::vec4{2.0f});
                    mPainter->DrawImage({0.25, 0.4, 0.7, 0.2}, 0.0f, mMainMenuLogo);
                    mPainter->DrawImage({0.97, 0.42, 0.16, 0.16}, angle, mMainMenuBall);

                    angle += step * dt;
                    angle = angle > 2.0f * pi? angle - 2.0f * pi: angle;
                }

                // Draw menu widget
                {
                    ImGui::Begin("Main Menu");

                    ImGui::Text(" - 1. Select the game scenario");
                    ImGui::ListBox("Game scenario", &selectedScenario, scenariosRaw.data(), scenariosRaw.size());
                    ImGui::NewLine();

                    ImGui::Text(" - 2. Select the algorithm used to control robots");
                    ImGui::ListBox("Algorithm", &selectedAlgo, algorithmsRaw.data(), algorithmsRaw.size());
                    ImGui::NewLine();

                    ImGui::Text(" - 3. Select game rules (leave empty for no rules)");
                    if (ImGui::BeginListBox("Rules")) {
                        for (int i = 0; i < rules.size(); i++) {
                             ImGui::Selectable(rules[i].data(), &selectedRules[i].v);
                        }

                        ImGui::EndListBox();
                    }
                    ImGui::NewLine();

                    ImGui::PushStyleColor(ImGuiCol_Button, mStyle.greenColor);
                    beginGame = ImGui::Button("Start", ImVec2(300, 0));

                    ImGui::PopStyleColor(1);
                    ImGui::End();
                }

                if (beginGame && selectedScenario >= 0 && selectedAlgo >= 0) {
                    mState = State::BeginGame;
                    needRefresh = true;
                }
            }
            else if (mState == State::BeginGame) {
                game = mGameManager->CreateGame(selectedScenario);
                algo = mAlgorithmManager->GetAlgorithmAt(selectedAlgo);

                rule->Clear();
                for (int i = 0; i < rules.size(); i++) {
                    if (selectedRules[i].v)
                        rule->AddRule(mGameRulesManager->GetRule(i));
                }
                ruleTextInfo = std::move(rule->GetName());

                mPhysicsServer->SetGameProperties(game->physicsGameProperties);
                mPhysicsServer->BeginGame(game->physicsGameInitInfo);
                mGraphicsServer->BeginGame(game->graphicsSceneSettings);
                algo->BeginGame(*game);

                t = 0.0f;

                mPhysicsServer->GetCurrentGameState(game->physicsGameState);

                mState = State::InGame;
                gameState = GameState::Paused;
            }
            else if (mState == State::InGame) {
                // Update loop strategy:
                // 1) Update physics - simulate game with step (step is prev frame delta)
                // 2) Draw game state and gui
                // 3) Tick algorithm control (if required)
                // 4) Update physics settings (motors power) (if required)

                auto simDt = gameState == GameState::Running? dt: 0.0f;
                auto timeScale = menuBar.timeScale;

                // Scale sim time
                simDt *= timeScale;

                if (gameState == GameState::Running) {

                    // Return true if must continue physics sim step
                    auto onFixedStep = [&] (float fixedDt) {
                        auto res = rule->Process(t, fixedDt, *game);

                        if (res == GameMessage::Finish) {
                            gameState = GameState::Finished;
                            return false;
                        }

                        algo->TickGame(fixedDt, t, *game);
                        t += fixedDt;

                        return true;
                    };

                    mPhysicsServer->FrameStep(game, onFixedStep, simDt);
                }

                mPainter->FitToFramebufferArea();

                mGraphicsServer->BeginDraw(simDt, dt, game->physicsGameState);
                mGraphicsServer->DrawStaticObjects();
                mGraphicsServer->DrawDynamicObjects();
                mGraphicsServer->DrawAuxInfo();
                mGraphicsServer->DrawPostUI();
                mGraphicsServer->EndDraw();

                // Draw Control window
                {
                    ImGui::Begin("Control Window", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

                    ImGui::BeginGroup();
                    ImGui::Text("Simulation:");
                    ImGui::Text(" - Time: %.2f sec", t);
                    ImGui::Text(" - Framerate: %.2f ms (%.2f)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
                    ImGui::Text(" - Status: %s", GameStateToStr(gameState));
                    ImGui::Text(" - Score: %u : %u", game->teamScoreA, game->teamScoreB);
                    ImGui::EndGroup();

                    ImGui::Separator();

                    ImGui::BeginGroup();
                    ImGui::Text("Targets:");
                    ImGui::Text(" - Scenario: %s", scenariosRaw[selectedScenario]);
                    ImGui::Text(" - Algorithm: %s", algorithmsRaw[selectedAlgo]);
                    ImGui::Text(" - Rules: %s", ruleTextInfo.c_str());
                    ImGui::EndGroup();

                    ImGui::Separator();

                    ImGui::PushStyleColor(ImGuiCol_Button, mStyle.redColor);
                    if (ImGui::Button("To Menu", ImVec2(130, 0))) {
                        gameState = GameState::Finished;
                        mState = State::EndGame;
                    }

                    ImGui::SameLine();

                    ImGui::PushStyleColor(ImGuiCol_Button, mStyle.violetColor);
                    if (ImGui::Button("Restart", ImVec2(130, 0))) {
                        gameState = GameState::Finished;
                        needRestart = true;
                        mState = State::EndGame;
                    }

                    ImGui::SameLine();

                    ImGui::PushStyleColor(ImGuiCol_Button, mStyle.greenColor);
                    if (ImGui::Button("Play", ImVec2(130, 0)) && gameState != GameState::Finished) {
                        gameState = GameState::Running;
                    }

                    ImGui::SameLine();

                    ImGui::PushStyleColor(ImGuiCol_Button, mStyle.yellowColor);
                    if (ImGui::Button("Pause", ImVec2(130, 0)) && gameState == GameState::Running) {
                        gameState = GameState::Paused;
                    }

                    ImGui::PopStyleColor(4);
                    ImGui::End();
                }

                // Draw debug info window if required
                if (menuBar.showDebugRobotInfo) {
                    ImGui::Begin("Debug Info");

                    auto& ball = game->physicsGameState.ball;
                    ImGui::Text("Ball: pos=(x=%f m,y=%f m) vel=(x=%f m/s, y=%f m/s)",
                                ball.position.x, ball.position.y, ball.velocity.x, ball.velocity.y);
                    ImGui::Separator();

                    ImGui::Text("Team A");
                    for (int i = 0; i < game->teamSize; i++) {
                        auto id = game->physicsGameInitInfo.robotsTeamA[i].id;
                        auto& r = game->physicsGameState.robots[id];
                        auto& p = game->robotWheelVelocitiesA[i];

                        ImGui::Text(" - Robot [%i]: pos=(x=%f m,y=%f m) vel=(x=%f m/s, y=%f m/s) angle=(%f rad) velWheels=(left %f m/s, right %f m/s)",
                                    id, r.position.x, r.position.y, r.velocity.x, r.velocity.y, r.angle, p.x, p.y);
                    }
                    ImGui::Separator();

                    ImGui::Text("Team B");
                    for (int i = 0; i < game->teamSize; i++) {
                        auto id = game->physicsGameInitInfo.robotsTeamB[i].id;
                        auto& r = game->physicsGameState.robots[id];
                        auto& p = game->robotWheelVelocitiesB[i];
                        
                        ImGui::Text(" - Robot [%i]: pos=(x=%f m,y=%f m) vel=(x=%f m/s, y=%f m/s) angle=(%f rad) velWheels=(left %f m/s, right %f m/s)",
                                    id, r.position.x, r.position.y, r.velocity.x, r.velocity.y, r.angle, p.x, p.y);
                    }
                    ImGui::Separator();

                    ImGui::End();
                }
            }
            else if (mState == State::EndGame) {
                algo->EndGame(*game);
                mGraphicsServer->EndGame();
                mPhysicsServer->EndGame();

                game = nullptr;
                algo = nullptr;

                if (needRestart) {
                    mState = State::BeginGame;
                    needRestart = false;
                }
                else {
                    mState = State::MainMenu;
                }
            }

            // Rendering
            ImGui::Render();
            mPainter->Draw();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

            // Now we are ready to present images
            mWindowManager->SwapBuffers();
        }

        // imgui clean-up
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();

        return 0;
    }

    const char * GuiSimulator::GameStateToStr(GameState state) {
        switch (state) {
            case GameState::Running:
                return "Running";
            case GameState::Finished:
                return "Finished";
            case GameState::Paused:
                return "Paused";
            default:
                return "(unknown)";
        }
    }


}