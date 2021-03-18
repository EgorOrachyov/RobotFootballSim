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
#include <physics/PhysicsServer.hpp>

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
        mPhysicsServer = std::make_shared<PhysicsServer>();
    }

    Simulator::~Simulator() {
        // Release in reverse order
        mPhysicsServer = nullptr;
        mPainter = nullptr;
        mPrimaryWindow = nullptr;
        mWindowManager = nullptr;
    }

    int Simulator::Run() {
        const float pi = glm::pi<float>();

        // TODO: change properties to real ones
        PhysicsGameProperties physicsProperties = {};
        {
            physicsProperties.fieldFriction = 0.5f;

            physicsProperties.robotRadius = 0.1f;
            physicsProperties.robotHeight = 0.1f;
            physicsProperties.robotMass = 1.0f;
            physicsProperties.robotFriction = 0.25f;
            physicsProperties.robotRestitution = 0.1f;
            // TODO: set correct values
            physicsProperties.robotLeftMotorOffset = { 0, -0.8f };
            physicsProperties.robotRightMotorOffset = { 0, 0.8f };

            physicsProperties.ballRadius = 0.05f;
            physicsProperties.ballMass = 0.250f;
            physicsProperties.ballFriction = 0.05f;
            physicsProperties.ballRestitution = 0.75f;   
        }

        mPhysicsServer->SetGameProperties(physicsProperties);


        PhysicsGameInitInfo beginInfo = {};
        beginInfo.fieldTopLeftBounds     = { 0, 0 };
        beginInfo.fieldBottomRightBounds = { 16,  9 };
        beginInfo.roomTopLeftBounds      = { -1, -1 };
        beginInfo.roomBottomRightBounds  = { 17,  10 };

        beginInfo.ballPosition = { 0, 0 };

        for (int i = 0; i < 6; i++)
        {
            beginInfo.robotsTeamA.push_back({ i,     { 16.0f * 0.5f - 4, 9 * 0.5f + (i - 2.5f) }, 0 });
            beginInfo.robotsTeamB.push_back({ i + 6, { 16.0f * 0.5f + 4, 9 * 0.5f + (i - 2.5f) }, pi });
        }

        mPhysicsServer->BeginGame(beginInfo);


        auto prefix = std::string("../..");
        auto robot = Image::LoadFromFilePath(prefix + "/resources/sprites/robot.png");
        auto shadow = Image::LoadFromFilePath(prefix + "/resources/sprites/shadow.png");
        auto field = Image::LoadFromFilePath(prefix + "/resources/sprites/play-field.png");

        auto white = glm::vec4(1.0f);
        auto noTrsp = glm::vec4(2.0f);

        mPainter->SetPenColor(white);
        mPainter->SetBrushColor(glm::vec4 {1.0f});
        mPainter->SetClearColor(glm::vec4 {0.1f});

        float dt = 1.0f / 60.0f;

        int frameCount = 0;

        while (!mPrimaryWindow->ShouldClose()) {

            mPhysicsServer->UpdateMotorsPower(0,25,50);

            mPhysicsServer->GameStep(dt);

            PhysicsGameState physicsState;
            mPhysicsServer->GetCurrentGameState(physicsState);

            mPainter->Clear();

            mPainter->SetDrawArea({ 0, 0, 16 * 80, 9 * 80 });
            mPainter->SetDrawSpace({ 0, 0, 16, 9 });

            mPainter->SetTransparentColor(noTrsp);
            mPainter->DrawImage({ 0, 0, 16, 9 }, 0, field);

            mPainter->SetBrushColor(white);
            mPainter->SetTransparentColor(white);

            for (const auto &r: physicsState.robots) {
                mPainter->DrawImage({r.position.x, r.position.y, physicsProperties.robotRadius * 2, physicsProperties.robotRadius * 2}, r.angle, robot);
            }


            mPainter->Draw();
            mWindowManager->Update();

            frameCount++;
        }

        mPhysicsServer->EndGame();

        return 0;
    }

}
