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

#include <graphics/GraphicsServer.hpp>
#include <glm/gtx/norm.hpp>
#include <iostream>
#include <sstream>
#include <random>
#include <chrono>

namespace rfsim {

    static constexpr auto WHITE_COLOR = glm::vec4(1.0f);
    static constexpr auto NO_TRANSPARENT_COLOR = glm::vec4(2.0f);
    static constexpr auto SHADOW_COLOR = glm::vec4(0.1, 0.1, 0.1, 1.5f);

    GraphicsServer::GraphicsServer(const std::shared_ptr<Window> &window,
                                   const std::shared_ptr<Painter> &painter,
                                   const std::string& resPath) {
        mWindow = window;
        mPainter = painter;
        mResPath = resPath;

        static const auto MARKER_IMAGE_PATH = "marker.png";
        static const auto ROBOT_IMAGE_PATH = "robot.png";
        static const auto FIELD_IMAGE_PATH = "play-field-div-b.png";
        static const auto BALL_IMAGE_PATH = "ball.png";
        static const auto BALL_IMAGE_TRSP_PATH = "soccer-ball.png";
        static const auto SHADOW_IMAGE_PATH = "shadow.png";
        static const auto ON_COLLISION_IMAGE_PATH = "hit.png";
        static const auto ON_OUT_IMAGE_PATH = "out-of-bounds.png";
        static const auto TRACE_POINT_IMAGE_PATH = "trace-point.png";

        auto prefix = mResPath + "/sprites/";

        mMarkerImage = Image::LoadFromFilePath(prefix + MARKER_IMAGE_PATH);
        mTraceImage = Image::LoadFromFilePath(prefix + TRACE_POINT_IMAGE_PATH);
        mBallImage = Image::LoadFromFilePath(prefix + BALL_IMAGE_TRSP_PATH);
        mFieldImage = Image::LoadFromFilePath(prefix + FIELD_IMAGE_PATH);
        mOnCollisionImage = Image::LoadFromFilePath(prefix + ON_COLLISION_IMAGE_PATH);
        mOnOutImage = Image::LoadFromFilePath(prefix + ON_OUT_IMAGE_PATH);
        mShadowImage = Image::LoadFromFilePath(prefix + SHADOW_IMAGE_PATH);

        static const  size_t TOTAL_ROBOTS = 16;
        static const  size_t TOTAL_NUMBERS= 16;

        for (size_t i = 0; i < TOTAL_ROBOTS; i++) {
            std::stringstream path;

            path << prefix << "robot_" << i << ".png";
            mRobotImages.push_back(Image::LoadFromFilePath(path.str()));
        }

        for (size_t i = 0; i < TOTAL_NUMBERS; i++) {
            std::stringstream path;

            path << prefix << "number_" << i << ".png";
            mRobotNumbers.push_back(Image::LoadFromFilePath(path.str()));
        }
    }

    void GraphicsServer::SetSettings(const GraphicsSettings &settings) {
        mSettings = settings;
    }

    void GraphicsServer::GetSettings(GraphicsSettings &settings) const {
        settings = mSettings;
    }

    void GraphicsServer::BeginGame(const GraphicsSceneSettings &sceneSettings) {
        assert(mState == InternalState::Default);
        mState = InternalState::InGame;

        mSceneSettings = sceneSettings;

        auto totalRobots = mSceneSettings.robotsTeamA.size() + mSceneSettings.robotsTeamB.size();

        mRobotsMarkers.resize(totalRobots);
        for (auto& marker: mRobotsMarkers) {
            marker = 0;
        }

        mRobotsTrace.resize(totalRobots, circular_buffer<glm::vec2>(mSettings.traceLength));

        for (auto& r: mSceneSettings.robotsTeamA) {
            mRobotsTrace[r.id].push_back(r.position);
        }

        for (auto& r: mSceneSettings.robotsTeamB) {
            mRobotsTrace[r.id].push_back(r.position);
        }

        mBallTrace.resize(mSettings.traceLength);
        mBallTrace.push_back(mSceneSettings.ballPosition);

        mTime = 0.0f;
        mMarkerOffset = 0.0f;
        mBallAngleAccum = 0.0f;
        mTimeLastTraceCapture = 0.0f;

        if (totalRobots > mRobotImages.size()) {
            std::cerr << "Too much robots on the scene (not enough sprites to display all robots with unique look)" << std::endl;
        }

        if (totalRobots > mRobotNumbers.size()) {
            std::cerr << "Too much robots on the scene (not enough sprites to display all robots with unique ids)" << std::endl;
        }
    }

    void GraphicsServer::BeginDraw(float simDt, float realDt, const GraphicsGameState &gameState) {
        assert(mState == InternalState::InGame);
        mState = InternalState::InGameBeginDraw;
        mCurrentState = gameState;
        mTime += simDt;

        auto room00 = mSceneSettings.roomTopLeftBounds;
        auto roomWH = mSceneSettings.roomBottomRightBounds;
        auto size = roomWH - room00;

        // Prepare area
        mPainter->SetClearColor({ mSettings.backgroundColor, 1.0f });
        mPainter->Clear();
        mPainter->SetDrawSpace({ room00, size });

        // Update trace
        if (mTime >= mTimeLastTraceCapture + mSettings.traceSkip) {
            for (size_t i = 0; i < gameState.robots.size(); i++) {
                mRobotsTrace[i].push_back(gameState.robots[i].position);
            }
            mBallTrace.push_back(gameState.ball.position);
            mTimeLastTraceCapture = mTime;
        }

        // Update markers offsets
        mMarkerOffset += realDt * mMarkerOffsetSpeed;
    }

    void GraphicsServer::DrawStaticObjects() {
        assert(mState == InternalState::InGameBeginDraw);

        auto room00 = mSceneSettings.roomTopLeftBounds;
        auto roomWH = mSceneSettings.roomBottomRightBounds;
        auto size = roomWH - room00;

        mPainter->SetTransparentColor(NO_TRANSPARENT_COLOR);
        mPainter->SetBrushColor({mSettings.fieldCustomColor, 1.0f});
        mPainter->DrawImage({room00, size}, 0, mFieldImage);
    }

    void GraphicsServer::DrawDynamicObjects() {
        assert(mState == InternalState::InGameBeginDraw);

        // Draw trace
        if (mSettings.drawTrace) {
            // Robots trace
            {
                auto r = mSettings.tracePointRadius * mSceneSettings.robotRadius;

                for (const auto& tr: mRobotsTrace) {
                    float factor = 0.1f;
                    float step = 0.9f / (float) mSettings.traceLength;

                    tr.for_each([&](const glm::vec2& pos) {
                        mPainter->SetBrushColor({mSettings.traceColor, factor});
                        mPainter->DrawImage({pos.x - r, pos.y - r, 2.0f * r, 2.0f * r}, 0.0f, mTraceImage);

                        factor += step;
                    });
                }
            }

            // Draw ball trace
            {
                auto r = mSettings.tracePointRadius * mSceneSettings.ballRadius;

                float factor = 0.1f;
                float step = 0.9f / (float) mSettings.traceLength;

                mBallTrace.for_each([&](const glm::vec2& pos) {
                    mPainter->SetBrushColor({mSettings.traceColor, factor});
                    mPainter->DrawImage({pos.x - r, pos.y - r, 2.0f * r, 2.0f * r}, 0.0f, mTraceImage);

                    factor += step;
                });
            }
        }

        // Draw shadow function
        auto drawRobotShadow = [&](const BodyState& r) {
            const auto radius = mSceneSettings.robotRadius;

            auto shadowRadius = radius * 1.1;
            const Painter::Rect shadowRect = {
                    r.position.x - shadowRadius,
                    r.position.y - shadowRadius,
                    shadowRadius * 2.0f,
                    shadowRadius * 2.0f
            };

            mPainter->SetBrushColor(mSettings.shadowIntensity * SHADOW_COLOR);
            mPainter->SetTransparentColor(NO_TRANSPARENT_COLOR);
            mPainter->DrawImage(shadowRect + glm::vec4{mSettings.sunPosition * shadowRadius,shadowRadius * 0.3,0,0}, 0, mShadowImage);
        };

        // Draw robot function
        auto drawRobot = [&](const BodyState& r, size_t id) {
            const auto radius = mSceneSettings.robotRadius;
            auto usedId = std::min(id, mRobotImages.size() - 1);
            auto texture = mRobotImages[usedId];

            const Painter::Rect rect = {
                r.position.x - radius,
                r.position.y - radius,
                radius * 2.0f,
                radius * 2.0f
            };

            mPainter->SetBrushColor(WHITE_COLOR);
            mPainter->SetTransparentColor(WHITE_COLOR);
            mPainter->DrawImage(rect, r.angle, texture);
        };

        // Draw robots shadows
        if (mSettings.drawShadows) {
            for (const auto& r: mCurrentState.robots) {
                drawRobotShadow(r);
            }
        }

        // Draw team A robots
        for (const auto& r: mSceneSettings.robotsTeamA) {
            auto id = r.id;
            drawRobot(mCurrentState.robots[id], id);
        }

        // Draw team B robots
        for (const auto& r: mSceneSettings.robotsTeamB) {
            auto id = r.id;
            drawRobot(mCurrentState.robots[id], id);
        }

        // Draw ball
        {
            const auto &b = mCurrentState.ball;
            const auto radius = mSceneSettings.ballRadius;

            const Painter::Rect rect = {
                    b.position.x - radius,
                    b.position.y - radius,
                    radius * 2.0f,
                    radius * 2.0f
            };

            if (mSettings.drawShadows) {
                mPainter->SetBrushColor(mSettings.shadowIntensity * SHADOW_COLOR);
                mPainter->SetTransparentColor(NO_TRANSPARENT_COLOR);
                mPainter->DrawImage(rect + glm::vec4{mSettings.sunPosition * radius,radius * 0.8,0,0}, 0, mShadowImage);
            }

            // Cool hack to make ball rotate
            float direction = b.velocity.x >= 0.0f? 1.0f: -1.0f;
            float magnitude = std::min(glm::length(b.velocity), 6.0f);
            float angle = direction * magnitude * 0.25f;

            mBallAngleAccum += angle;

            mPainter->SetBrushColor(WHITE_COLOR);
            mPainter->SetTransparentColor(NO_TRANSPARENT_COLOR);
            mPainter->DrawImage(rect, mBallAngleAccum, mBallImage);
        }
    }

    void GraphicsServer::DrawAuxInfo() {
        assert(mState == InternalState::InGameBeginDraw);

        // Draw info about collisions on top of the robots
        if (mSettings.drawCollisionInfo) {
            for (const auto &c : mCurrentState.robotRobotCollisions) {
                const float factor = 1.0f;
                const float size = mSceneSettings.robotRadius * 2 * factor;

                const auto &ra = mCurrentState.robots[c.robotIdA];
                const auto &rb = mCurrentState.robots[c.robotIdB];

                const Painter::Rect rectA = {
                    ra.position.x - size / 2,
                    ra.position.y - size / 2,
                    size, size
                };

                const Painter::Rect rectB = {
                    rb.position.x - size / 2,
                    rb.position.y - size / 2,
                    size, size
                };

                mPainter->DrawImage(rectA, 0, mOnCollisionImage);
                mPainter->DrawImage(rectB, 0, mOnCollisionImage);
            }
        }

        // Draw info about robots, which leave the area of the field
        if (mSettings.drawOutInfo) {
            for (int id : mCurrentState.robotFieldBoundsCollisions) {
                const float factor = 2.0f;
                const float size = mSceneSettings.robotRadius * 2 * factor;

                const auto &r = mCurrentState.robots[id];

                const Painter::Rect rect = {
                    r.position.x - size / 2.0f,
                    r.position.y - size / 2.0f,
                    size, size
                };

                mPainter->DrawImage(rect, 0, mOnOutImage);
            }
        }

        // Draw Robots Ids
        if (mSettings.drawRobotIDs) {
            for (size_t i = 0; i < mCurrentState.robots.size(); i++) {
                auto actualId = std::min(i, mRobotNumbers.size() - 1);
                auto number = mRobotNumbers[actualId];
                auto& r = mCurrentState.robots[i];
                const auto radius = mSceneSettings.robotRadius;

                const Painter::Rect rect = {
                        r.position.x - radius * 2.0,
                        r.position.y - radius * 1.5,
                        radius * 1.5,
                        radius * 1.5
                };

                mPainter->SetBrushColor(WHITE_COLOR);
                mPainter->SetTransparentColor(NO_TRANSPARENT_COLOR);
                mPainter->DrawImage(rect, 0.0f, number);
            }
        }

        // Draw Robots Markers
        if (mSettings.drawMarkers) {
            const float radius = mSceneSettings.robotRadius;
            const float markerSize = mSettings.markerProp * radius;
            const float offset = 1.0f + std::sin(mMarkerOffset);
            const float displacement = offset * markerSize * mMarkerOffsetScale + markerSize;

            auto drawMarker = [&](size_t id, const glm::vec3& color) {
                const auto& r = mCurrentState.robots[id];


                const Painter::Rect rect = {
                    r.position.x - markerSize * 0.5f,
                    r.position.y - radius - displacement,
                    markerSize,
                    markerSize
                };

                mPainter->SetBrushColor({color, mSettings.markerOpacity});
                mPainter->SetTransparentColor(NO_TRANSPARENT_COLOR);
                mPainter->DrawImage(rect, 0.0f, mMarkerImage);
            };

            // Draw team A markers
            for (const auto& r: mSceneSettings.robotsTeamA) {
                drawMarker(r.id, mSettings.team1Color);
            }

            // Draw team B markers
            for (const auto& r: mSceneSettings.robotsTeamB) {
                drawMarker(r.id, mSettings.team2Color);
            }
        }
    }

    void GraphicsServer::DrawPostUI() {
        assert(mState == InternalState::InGameBeginDraw);

        // Draw something, what must be placed on top of the UI
    }

    void GraphicsServer::EndDraw() {
        assert(mState == InternalState::InGameBeginDraw);
        mState = InternalState::InGame;

        // Commit draw
        mPainter->Draw();
    }

    void GraphicsServer::EndGame() {
        assert(mState == InternalState::InGame);
        mState = InternalState::Default;
        mRobotsTrace.clear();
    }

}