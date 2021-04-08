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

#ifndef RFSIM_GRAPHICSSERVER_HPP
#define RFSIM_GRAPHICSSERVER_HPP

#include <graphics/Image.hpp>
#include <graphics/Window.hpp>
#include <graphics/Painter.hpp>
#include <graphics/GraphicsSettings.hpp>
#include <graphics/GraphicsGameState.hpp>
#include <graphics/GraphicsSceneSettings.hpp>
#include <physics/PhysicsGameInitInfo.hpp>
#include <physics/PhysicsGameProperties.hpp>
#include <physics/PhysicsGameState.hpp>
#include <utils/CircularBuffer.hpp>

namespace rfsim {

    class GraphicsServer {
    public:
        GraphicsServer(const std::shared_ptr<Window> &window, const std::shared_ptr<Painter> &painter, const std::string& resPath);
        GraphicsServer(const GraphicsServer& other) = delete;
        GraphicsServer(GraphicsServer&& other) noexcept = delete;
        ~GraphicsServer() = default;

        void SetSettings(const GraphicsSettings& settings);
        void GetSettings(GraphicsSettings& settings) const;

        void BeginGame(const GraphicsSceneSettings& sceneSettings);
        void BeginDraw(float simDt, float realDt, const GraphicsGameState &gameState);
        void DrawStaticObjects();
        void DrawDynamicObjects();
        void DrawAuxInfo();
        void DrawPostUI();
        void EndDraw();
        void EndGame();

    private:

        enum class InternalState {
            Default,
            InGame,
            InGameBeginDraw
        };

        InternalState mState = InternalState::Default;

        GraphicsSettings mSettings;
        GraphicsSceneSettings mSceneSettings;
        GraphicsGameState mCurrentState;

        std::string mResPath;
        std::shared_ptr<Window> mWindow;
        std::shared_ptr<Painter> mPainter;

        std::shared_ptr<Image> mMarkerImage;
        std::shared_ptr<Image> mTraceImage;
        std::shared_ptr<Image> mBallImage;
        std::shared_ptr<Image> mFieldImage;
        std::shared_ptr<Image> mOnCollisionImage;
        std::shared_ptr<Image> mOnOutImage;
        std::shared_ptr<Image> mShadowImage;
        std::vector<std::shared_ptr<Image>> mRobotImages;
        std::vector<std::shared_ptr<Image>> mRobotNumbers;

        circular_buffer<glm::vec2> mBallTrace;
        std::vector<circular_buffer<glm::vec2>> mRobotsTrace;
        std::vector<float> mRobotsMarkers;

        float mTime = 0.0f;
        float mMarkerOffset = 0.0f;
        float mMarkerOffsetScale = 0.8f;
        float mMarkerOffsetSpeed = 3.0f;
        float mBallAngleAccum = 0.0f;
        float mTimeLastTraceCapture = 0.0f;
    };

}

#endif //RFSIM_GRAPHICSSERVER_HPP