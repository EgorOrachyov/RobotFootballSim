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
#include <glm/gtc/constants.hpp>
#include <imgui.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>
#include <memory>

namespace rfsim {

    GuiSimulator::GuiSimulator(int argc, const char* const* argv) : Simulator(argc, argv) {
        const auto SEP = "/";
        const auto prefix = mResourcesPath + SEP + "sprites" + SEP;

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
        io.FontGlobalScale *= mFontScale;

        auto& style = ImGui::GetStyle();
        style.ScaleAllSizes(mGuiScale);

        ImGui::StyleColorsDark();
        ImGui_ImplGlfw_InitForOpenGL(mPrimaryWindow->GetNativeHnd(), true);
        ImGui_ImplOpenGL3_Init(glsl_version);

        // Global simulator state
        GuiMenuBar menuBar;
        GameState gameState;
        std::shared_ptr<Game> game;
        std::shared_ptr<Algorithm> algo;

        // This is main menu related data
        bool needRefresh = true;
        bool beginGame;
        bool exit = false;
        int selectedScenario = -1;
        int selectedAlgo = -1;
        std::vector<std::string> scenarios;
        std::vector<const char*> scenariosRaw;
        std::vector<std::string> algorithms;
        std::vector<const char*> algorithmsRaw;

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

                    mAlgorithmManager->GetAlgorithmsInfo(algorithms);
                    mGameManager->GetScenarioInfo(scenarios);

                    scenariosRaw.clear();
                    scenariosRaw.reserve(scenarios.size());

                    for (auto& s: scenarios)
                        scenariosRaw.push_back(s.data());

                    algorithmsRaw.clear();
                    algorithmsRaw.reserve(algorithms.size());

                    for (auto& a: algorithms)
                        algorithmsRaw.push_back(a.data());

                    needRefresh = false;
                }

                // Draw animated background logo
                {
                    static const auto pi = glm::pi<float>();
                    static float angle = 0.0f;
                    static float step = 0.01f;

                    auto size = mPrimaryWindow->GetFramebufferSize();
                    auto aspect = (float) size.x / (float) size.y;

                    mPainter->Clear();
                    mPainter->SetDrawArea({0, 0, size});
                    mPainter->SetDrawSpace({0, 0, aspect, 1});
                    mPainter->SetClearColor({mStyle.clearColor.x, mStyle.clearColor.y, mStyle.clearColor.z, mStyle.clearColor.w});
                    mPainter->SetBrushColor(glm::vec4 {1.0f});
                    mPainter->SetTransparentColor(glm::vec4{2.0f});
                    mPainter->DrawImage({0.45, 0.4, 0.7, 0.2}, 0.0f, mMainMenuLogo);
                    mPainter->DrawImage({1.17, 0.42, 0.16, 0.16}, angle, mMainMenuBall);

                    angle += step;
                    angle = angle > 2.0f * pi? angle - 2.0f * pi: angle;
                }

                // Draw menu widget
                ImGui::Begin("Main Menu");

                ImGui::Text("How to start a new game:");
                ImGui::Text(" - 1. Select the game scenario");
                ImGui::Text(" - 2. Select the algorithm used to control robots");
                ImGui::Text(" - 3. Press start button");
                ImGui::NewLine();

                ImGui::ListBox("Game scenario", &selectedScenario, scenariosRaw.data(), scenariosRaw.size());
                ImGui::ListBox("Algorithm", &selectedAlgo, algorithmsRaw.data(), algorithmsRaw.size());
                ImGui::NewLine();

                ImGui::PushStyleColor(ImGuiCol_Button, mStyle.redColor);
                exit |= ImGui::Button("Quit", ImVec2(200, 0));
                ImGui::SameLine();
                ImGui::PushStyleColor(ImGuiCol_Button, mStyle.greenColor);
                beginGame = ImGui::Button("Start", ImVec2(200, 0));

                ImGui::PopStyleColor(2);
                ImGui::End();

                if (beginGame && selectedScenario >= 0 && selectedAlgo >= 0) {
                    mState = State::BeginGame;
                    needRefresh = true;
                }
            }
            else if (mState == State::BeginGame) {
                game = mGameManager->CreateGame(selectedScenario);
                algo = mAlgorithmManager->GetAlgorithmAt(selectedAlgo);

                mPhysicsServer->SetGameProperties(game->physicsGameProperties);
                mPhysicsServer->BeginGame(game->physicsGameInitInfo);
                mGraphicsServer->BeginGame(game->graphicsSceneSettings);
                algo->BeginGame(*game);

                t = 0.0f;

                PhysicsGameState state;
                mPhysicsServer->GetCurrentGameState(state);
                game->physicsGameState = state;

                mState = State::InGame;
                gameState = GameState::Paused;
            }
            else if (mState == State::InGame) {
                // Update loop strategy:
                // 1) Update physics - simulate game with step (step is prev frame delta)
                // 2) Draw game state and gui
                // 3) Tick algorithm control (if required)
                // 4) Update physics settings (motors power) (if required)

                // Update sim delta time
                dt = 1.0f / ImGui::GetIO().Framerate;
                dt = dt > (1.0f / 30.f)? 1.0f / 30.0f: dt;

                if (gameState == GameState::Running) {
                    t += dt;

                    PhysicsGameState state;
                    mPhysicsServer->GameStep(dt);
                    mPhysicsServer->GetCurrentGameState(state);

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
                }

                mPainter->FitToFramebufferArea();

                mGraphicsServer->BeginDraw(game->physicsGameState);
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
                    ImGui::EndGroup();

                    ImGui::Separator();

                    ImGui::PushStyleColor(ImGuiCol_Button, mStyle.redColor);
                    if (ImGui::Button("Quit Game", ImVec2(200, 0))) {
                        gameState = GameState::Finished;
                        mState = State::EndGame;
                    }

                    ImGui::SameLine();

                    ImGui::PushStyleColor(ImGuiCol_Button, mStyle.violetColor);
                    if (ImGui::Button("Restart", ImVec2(200, 0))) {
                        gameState = GameState::Finished;
                        needRestart = true;
                        mState = State::EndGame;
                    }

                    ImGui::SameLine();

                    ImGui::PushStyleColor(ImGuiCol_Button, mStyle.greenColor);
                    if (ImGui::Button("Play", ImVec2(200, 0)) && gameState != GameState::Finished) {
                        gameState = GameState::Running;
                    }

                    ImGui::SameLine();

                    ImGui::PushStyleColor(ImGuiCol_Button, mStyle.yellowColor);
                    if (ImGui::Button("Pause", ImVec2(200, 0)) && gameState == GameState::Running) {
                        gameState = GameState::Paused;
                    }

                    ImGui::PopStyleColor(4);
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