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
        const auto pi = glm::pi<float>();

        const float fieldLength = 16;
        const float fieldWidth = 9;

        // TODO: change properties to real ones
        PhysicsGameProperties physicsProperties = {};
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
        physicsProperties.ballMass = 0.05f;
        physicsProperties.ballFriction = 0.01f;
        physicsProperties.ballRestitution = 0.75f;   

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

        mPhysicsServer->BeginGame(beginInfo);

        auto prefix = std::string("../..");
        auto ballImg = Image::LoadFromFilePath(prefix + "/resources/sprites/ball.png");
        auto robotImg = Image::LoadFromFilePath(prefix + "/resources/sprites/robot.png");
        auto hitImg = Image::LoadFromFilePath(prefix + "/resources/sprites/hit.png");
        auto outOfBoundsImg = Image::LoadFromFilePath(prefix + "/resources/sprites/out-of-bounds.png");
        auto shadowImg = Image::LoadFromFilePath(prefix + "/resources/sprites/shadow.png");
        auto fieldImg = Image::LoadFromFilePath(prefix + "/resources/sprites/play-field.png");

        auto white = glm::vec4(1.0f);
        auto noTrsp = glm::vec4(2.0f);

        mPainter->SetPenColor(white);
        mPainter->SetBrushColor(glm::vec4 {1.0f});
        mPainter->SetClearColor(glm::vec4 {0.1f});

        float dt = 1.0f / 60.0f;

        uint64_t frameCount = 0;

        while (!mPrimaryWindow->ShouldClose()) {

            for (int i = 0; i < beginInfo.robotsTeamA.size() * 2; i++) {
                mPhysicsServer->UpdateMotorsPower(i, 50 * (float)rand() / RAND_MAX, 50 * (float)rand() / RAND_MAX);
            }

            mPhysicsServer->GameStep(dt);

            PhysicsGameState physicsState;
            mPhysicsServer->GetCurrentGameState(physicsState);

            mPainter->Clear();
            mPainter->SetDrawSpace({ 0, 0, fieldLength, fieldWidth });

            mPainter->SetTransparentColor(noTrsp);
            mPainter->DrawImage({ 0, 0, fieldLength, fieldWidth }, 0, fieldImg);

            mPainter->SetBrushColor(white);
            mPainter->SetTransparentColor(white);

            for (const auto &r: physicsState.robots) {
                const float radius = physicsProperties.robotRadius;

                const PainterEngine::Rect rect = {
                    r.position.x - radius,
                    r.position.y - radius,
                    radius * 2, radius * 2
                };

                mPainter->DrawImage(rect, r.angle, robotImg);
            }

            const auto &b = physicsState.ball;
            mPainter->DrawImage({b.position.x-physicsProperties.ballRadius, b.position.y-physicsProperties.ballRadius, physicsProperties.ballRadius * 2, physicsProperties.ballRadius * 2}, b.angle, ballImg);

            for (const auto &c : physicsState.robotRobotCollisions) {
                const auto &ra = physicsState.robots[c.robotIdA];
                const auto &rb = physicsState.robots[c.robotIdB];
                const float size = physicsProperties.robotRadius * 4;

                const PainterEngine::Rect rectA = {
                    ra.position.x - size / 2,
                    ra.position.y - size / 2,
                    size, size
                };
                
                const PainterEngine::Rect rectB = {
                    rb.position.x - size / 2,
                    rb.position.y - size / 2,
                    size, size
                };

                mPainter->DrawImage(rectA, 0, hitImg);
                mPainter->DrawImage(rectB, 0, hitImg);
            }

            for (int id : physicsState.robotFieldBoundsCollisions) {
                const auto &r = physicsState.robots[id];
                const float size = physicsProperties.robotRadius * 4;

                const PainterEngine::Rect rect = {
                    r.position.x - size / 2,
                    r.position.y - size / 2,
                    size, size
                };

                mPainter->DrawImage(rect, 0, outOfBoundsImg);
            }

            mPainter->Draw();
            mWindowManager->Update();
            frameCount++;

            // Update draw area with actual window framebuffer size
            auto areaSize = mPrimaryWindow->GetFramebufferSize();
            mPainter->SetDrawArea({0, 0, areaSize[0], areaSize[1]});
        }

        mPhysicsServer->EndGame();

        return 0;
    }

}
