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

#include <gui/GuiApplication.hpp>
#include <glm/gtc/constants.hpp>
#include <imgui.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>
#include <iostream>

namespace rfsim {

    GuiApplication::GuiApplication(const std::shared_ptr<Window> &window,
                                   const std::shared_ptr<WindowManager> &winMan,
                                   const std::shared_ptr<Painter> &painter,
                                   const std::shared_ptr<PhysicsServer> &physics,
                                   const std::shared_ptr<GraphicsServer> &graphics,
                                   const std::shared_ptr<AlgorithmManager> &algoMan,
                                   const std::string& resPath) {

        mWindow = window;
        mWindowManager = winMan;
        mPainter = painter;
        mPhysicsServer = physics;
        mGraphicsServer = graphics;
        mAlgorithmManager = algoMan;

        mResPath = resPath;

        const auto SEP = "/";
        const auto prefix = resPath + SEP + "sprites" + SEP;

        mMainMenuLogo = Image::LoadFromFilePath(prefix + "main-menu-logo.png");
        mMainMenuBall = Image::LoadFromFilePath(prefix + "soccer-ball.png");
    }

    int GuiApplication::Run() {
        // Setup imgui
#if defined(__APPLE__)
        // GL 3.2 + GLSL 150
        const char* glsl_version = "#version 150";
#else
        // GL 3.0 + GLSL 130
        const char* glsl_version = "#version 130";
#endif

        IMGUI_CHECKVERSION();
        ImGui::CreateContext();

        auto& io = ImGui::GetIO();
        io.FontGlobalScale *= mFontScale;

        auto& style = ImGui::GetStyle();
        style.ScaleAllSizes(mGuiScale);

        ImGui::StyleColorsDark();
        ImGui_ImplGlfw_InitForOpenGL(mWindow->GetNativeHnd(), true);
        ImGui_ImplOpenGL3_Init(glsl_version);

        mMainMenu = std::make_shared<GuiMainMenu>(*this);
        mMainMenu->Refresh();

        bool explicitlyExit = false;

        while (!mWindow->ShouldClose() && !explicitlyExit) {

            mWindowManager->UpdateEvents();

            auto size = mWindow->GetFramebufferSize();
            auto aspect = (float) size.x / (float) size.y;

            mPainter->SetDrawArea({0, 0, size});
            mPainter->SetDrawSpace({0, 0, aspect, 1});
            mPainter->SetClearColor({mStyle.clearColor.x, mStyle.clearColor.y, mStyle.clearColor.z, mStyle.clearColor.w});
            mPainter->Clear();

            // Start the Dear ImGui frame
            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();

            if (mState == GlobalState::MainMenu) {
                // Draw animated background logo
                {
                    static const auto pi = glm::pi<float>();
                    static float angle = 0.0f;
                    static float step = 0.01f;

                    mPainter->SetBrushColor(glm::vec4 {1.0f});
                    mPainter->SetTransparentColor(glm::vec4{2.0f});
                    mPainter->DrawImage({0.45, 0.4, 0.7, 0.2}, 0.0f, mMainMenuLogo);
                    mPainter->DrawImage({1.17, 0.42, 0.16, 0.16}, angle, mMainMenuBall);

                    angle += step;
                    angle = angle > 2.0f * pi? angle - 2.0f * pi: angle;
                }

                // Draw menu widget
                mMainMenu->Update();

                if (mMainMenu->BeginGame()) {
                    mState = GlobalState::PrepareGame;
                }

                if (mMainMenu->Exit()) {
                    explicitlyExit = true;
                }
            }

            if (mState == GlobalState::PrepareGame) {

                mState = GlobalState::MainMenu;
            }

            // Rendering
            ImGui::Render();
            mPainter->Draw();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

            mWindowManager->SwapBuffers();
        }

        // imgui clean-up
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();

        return 0;
    }

    GuiStyle& GuiApplication::GetStyle() {
        return mStyle;
    }

}