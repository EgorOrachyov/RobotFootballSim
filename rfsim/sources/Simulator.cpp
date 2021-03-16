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
#include <graphics/PainterEngine.hpp>
#include <graphics/WindowManager.hpp>
#include <cmath>
#include <glm/gtc/constants.hpp>

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
    }

    Simulator::~Simulator() {
        // Release in reverse order
        mPainter = nullptr;
        mPrimaryWindow = nullptr;
        mWindowManager = nullptr;
    }

    int Simulator::Run() {
        auto robot = Image::LoadFromFilePath("../../resources/sprites/robot.png");
        auto angle = 0.0f;
        auto radius = glm::vec2(500, 300);
        auto center = glm::vec2(640, 360);
        auto offset = glm::vec2(-50, -50);

        float pi = glm::pi<float>();
        float steps[4] = { pi * 0.0f, pi * 0.5f, pi * 1.0f, pi * 1.5f };

        mPainter->SetBrushColor({1.0f, 0.5f, 1.0f, 1.0f});
        mPainter->SetClearColor({0.1f, 0.2f, 0.1f, 0.0f});

        while (!mPrimaryWindow->ShouldClose()) {
            mPainter->Clear();

            for (float step: steps) {
                auto a = angle + step;
                auto pos = offset + center + radius * glm::vec2(std::cos(pi * 0.5f - a), -std::sin(pi * 0.5f - a));
                mPainter->DrawImage({pos.x, pos.y, 100, 100}, a, robot);
            }

            angle += 0.02f;

            mPainter->Draw();
            mWindowManager->Update();
        }

        return 0;
    }

}
