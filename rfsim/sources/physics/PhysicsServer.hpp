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

#ifndef RFSIM_PHYSICSSERVER_HPP
#define RFSIM_PHYSICSSERVER_HPP

#include <map>
#include <memory>

#include <physics/PhysicsGameProperties.hpp>
#include <physics/PhysicsGameInitInfo.hpp>
#include <physics/PhysicsGameState.hpp>

class b2World;
class b2Body;

namespace rfsim {

    class PhysicsContactListener;

    class PhysicsServer {
    public:
        PhysicsServer(float fixedDt = 1.0f / 60.0f);
        ~PhysicsServer();

        PhysicsServer(const PhysicsServer &other) = delete;
        PhysicsServer(PhysicsServer &&other) noexcept = delete;
        PhysicsServer& operator=(const PhysicsServer &other) = delete;
        PhysicsServer& operator=(PhysicsServer &&other) noexcept = delete;

        void SetGameProperties(const PhysicsGameProperties& properties);
        void BeginGame(const PhysicsGameInitInfo& info);
       
        void AccumulateDeltaTime(float dt);
        float GetFixedDt() const;
        bool TryGameStep();

        void UpdateWheelVelocities(int robotId, float leftWheelVelocity, float rightWheelVelocity);
        void GetCurrentGameState(PhysicsGameState& state) const;
        void EndGame();

    private:
        void SetFieldFriction(b2Body *target, float maxForceMult = 1.0f, float maxTorqueMult = 1.0f);

        bool IsRoomBounds(b2Body *body) const;
        bool IsFieldBounds(b2Body *body) const;
        bool IsGoal(b2Body *body) const;
        bool IsBall(b2Body *body) const;
        bool TryGetRobotByBody(b2Body *body, int &outId) const;

        // Convert position from game coordinates (y axis points down) to physics ones (y axis points up)
        static glm::vec2 ToPhysicsCoords(const glm::vec2& p);
        static float AngleToPhysicsCoords(float angle);
        // Convert position from physics coordinates (y axis points up) to game ones (y axis points down)
        static glm::vec2 ToGameCoords(const glm::vec2& p);
        static float AngleToGameCoords(float angle);

    private:
        float mFixedDt;
        float mDtAccumulated;

        std::shared_ptr<b2World> mWorld;
        std::shared_ptr<PhysicsContactListener> mContactListener;

        b2Body *mRoomBounds;
        b2Body *mFieldBoundSensors;

        // Each robot has its ID as the index in the vector
        std::map<int, b2Body*> mRobots;
        b2Body* mBall;

        PhysicsGameProperties mProperties;
    };

}

#endif //RFSIM_PHYSICSSERVER_HPP