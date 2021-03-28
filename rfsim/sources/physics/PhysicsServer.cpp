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

#include <physics/PhysicsServer.hpp>
#include <glm/ext/scalar_constants.hpp>
#include <box2d/box2d.h>
#include <algorithm>
#include <list>

namespace rfsim {

    class PhysicsContactListener : public b2ContactListener {
    public:
        PhysicsContactListener(const b2Body *fieldBoundSensors) : mFieldBoundSensors(fieldBoundSensors) {}
        ~PhysicsContactListener() override = default;

        PhysicsContactListener(const PhysicsContactListener &other) = delete;
        PhysicsContactListener(PhysicsContactListener &&other) noexcept = delete;
        PhysicsContactListener & operator=(const PhysicsContactListener &other) = delete;
        PhysicsContactListener & operator=(PhysicsContactListener &&other) noexcept = delete;

        // Sensor contacts are only passed to BeginContact/EndContact
        void BeginContact(b2Contact *contact) override {
            if (mFieldBoundSensors == contact->GetFixtureA()->GetBody() || 
                mFieldBoundSensors == contact->GetFixtureB()->GetBody()) {
                mSensorContacts.emplace_back(contact->GetFixtureA(), contact->GetFixtureB());
            }
        }

        void EndContact(b2Contact *contact) override {
            if (mFieldBoundSensors == contact->GetFixtureA()->GetBody() || 
                mFieldBoundSensors == contact->GetFixtureB()->GetBody()) {
                const std::pair<b2Fixture*, b2Fixture*> a = { contact->GetFixtureA(), contact->GetFixtureB() };

                mSensorContacts.erase(std::find(mSensorContacts.begin(), mSensorContacts.end(), a));
            }
        }

        void PreSolve(b2Contact* contact, const b2Manifold* oldManifold) override {
            const b2Manifold* manifold = contact->GetManifold();

	        if (manifold->pointCount == 0) {
		        return;
	        }

            mContacts.push_back({ contact->GetFixtureA(), contact->GetFixtureB() });
	    }

        void GetContacts(std::vector<std::pair<b2Fixture*, b2Fixture*>> &contacts) {
            for (const auto &c : mContacts) {
                contacts.emplace_back(c);
            }

            for (const auto &c : mSensorContacts) {
                contacts.emplace_back(c);
            }
        }

        void Clear() {
            mContacts.clear();
        }

    private:
        const b2Body *mFieldBoundSensors;

        std::vector<std::pair<b2Fixture*, b2Fixture*>> mContacts;
        std::list<std::pair<b2Fixture*, b2Fixture*>> mSensorContacts;
    };

    PhysicsServer::PhysicsServer() : mRoomBounds(nullptr), mFieldBoundSensors(nullptr), mBall(nullptr) {}

    PhysicsServer::~PhysicsServer() {
        EndGame();
    }

    void PhysicsServer::SetGameProperties(const PhysicsGameProperties &properties) {
        mProperties = properties;
    }

    void PhysicsServer::BeginGame(const PhysicsGameInitInfo &info) {
        assert(info.robotsTeamA.size() == info.robotsTeamB.size());
        assert(!mWorld);
        assert(!mContactListener);

        mWorld = std::make_shared<b2World>(b2Vec2(0, 0));

        const float roomFriction = 0.5f;
        const float roomRestitution = 0.5f;

        // physics engine uses different coordinates: y axis points up, while in game it points down
        const auto ballPosition           = ToPhysicsCoords(info.ballPosition);
        const auto roomTopLeftBounds      = ToPhysicsCoords(info.roomTopLeftBounds);
        const auto roomBottomRightBounds  = ToPhysicsCoords(info.roomBottomRightBounds);
        const auto fieldTopLeftBounds     = ToPhysicsCoords(info.fieldTopLeftBounds);
        const auto fieldBottomRightBounds = ToPhysicsCoords(info.fieldBottomRightBounds);

        // room left-right, top-bottom
        float rL = roomTopLeftBounds.x;
        float rR = roomBottomRightBounds.x;

        float rT = roomTopLeftBounds.y;
        float rB = roomBottomRightBounds.y;
        
        // field left-right, top-bottom
        float fL = fieldTopLeftBounds.x;
        float fR = fieldBottomRightBounds.x;

        float fT = fieldTopLeftBounds.y;
        float fB = fieldBottomRightBounds.y;

        // field must be smaller than room
        assert(rL <= fL);
        assert(rT >= fT);
        assert(rR >= fR);
        assert(rB <= fB);

        // create room bounds
        {
            b2BodyDef staticBodyDef = {};
            mRoomBounds = mWorld->CreateBody(&staticBodyDef);

            b2EdgeShape shape = {};

            b2FixtureDef commonFixture = {};
            commonFixture.shape = &shape;
            commonFixture.density = 0.0f;
            commonFixture.friction = roomFriction;
            commonFixture.restitution = roomRestitution;

            // left vertical
            shape.SetTwoSided(b2Vec2(rL, rT), b2Vec2(rL, rB));
		    mRoomBounds->CreateFixture(&commonFixture);

            // right vertical
            shape.SetTwoSided(b2Vec2(rR, rT), b2Vec2(rR, rB));
		    mRoomBounds->CreateFixture(&commonFixture);

            // top horizontal
            shape.SetTwoSided(b2Vec2(rL, rT), b2Vec2(rR, rT));
		    mRoomBounds->CreateFixture(&commonFixture);

            // bottom horizontal
            shape.SetTwoSided(b2Vec2(rL, rB), b2Vec2(rR, rB));
		    mRoomBounds->CreateFixture(&commonFixture);
        }

        // create field bounds
        {
            b2Vec2 leftCenter  = { rL, (rT + rB) * 0.5f };
            b2Vec2 rightCenter = { rR, (rT + rB) * 0.5f };

            b2Vec2 topCenter    = { (rR + rL) * 0.5f, rT };
            b2Vec2 bottomCenter = { (rR + rL) * 0.5f, rB };

            float leftDiff   = fL - rL;
            float topDiff    = rT - fT;
            float rightDiff  = rR - fR;
            float bottomDiff = fB - rB;

            float rWidth = rR - rL;
            float rHeight = rB - rT;

            b2BodyDef staticBodyDef = {};
            mFieldBoundSensors = mWorld->CreateBody(&staticBodyDef);
            
            b2PolygonShape shape = {};

            b2FixtureDef commonFixture = {};
            commonFixture.shape = &shape;
            commonFixture.density = 0.0f;
            // disable collisions, send only info about them
            commonFixture.isSensor = true;

            shape.SetAsBox(leftDiff, rHeight, leftCenter, 0);
            mFieldBoundSensors->CreateFixture(&commonFixture);
            
            shape.SetAsBox(rightDiff, rHeight, rightCenter, 0);
            mFieldBoundSensors->CreateFixture(&commonFixture);
            
            shape.SetAsBox(rWidth, topDiff, topCenter, 0);
            mFieldBoundSensors->CreateFixture(&commonFixture);
            
            shape.SetAsBox(rWidth, bottomDiff, bottomCenter, 0);
            mFieldBoundSensors->CreateFixture(&commonFixture);
        }

        mContactListener = std::make_shared<PhysicsContactListener>(mFieldBoundSensors);
        mWorld->SetContactListener(mContactListener.get());

        // ball
        {
            b2BodyDef dynamicBodyDef = {};
            dynamicBodyDef.type = b2_dynamicBody;
            dynamicBodyDef.position = { ballPosition.x, ballPosition.y };

            mBall = mWorld->CreateBody(&dynamicBodyDef);

            const auto r = (double)mProperties.ballRadius;
            const double ballVolume = 4.0 / 3.0 * glm::pi<double>() * r * r * r;
            assert(ballVolume > 0.0);

            b2CircleShape shape = {};
            shape.m_radius = r;

            b2FixtureDef fixture = {};
            fixture.density = (float)((double)mProperties.ballMass / ballVolume);
            fixture.friction = mProperties.ballFriction;
            fixture.restitution = mProperties.ballRestitution;
            fixture.shape = &shape;

            mBall->CreateFixture(&fixture);

            SetFieldFriction(mBall);
        }

        // robots
        const std::vector<RobotInitInfo>* allRobots[] = {
            &info.robotsTeamA,
            &info.robotsTeamB
        };

        const float robotAngularDamping = 100;

        for (const auto *lst : allRobots) {
            for (const auto &i : *lst) {
                // robot IDs must be less than robot count
                assert(i.id < info.robotsTeamA.size() + info.robotsTeamB.size());

                // must not exist with the same id
                assert(mRobots.find(i.id) == mRobots.end());

                const glm::vec2 pos = ToPhysicsCoords({ i.position.x, i.position.y });
                const float angle = AngleToPhysicsCoords(i.angle);

                b2BodyDef robotBodyDef = {};
                robotBodyDef.type = b2_dynamicBody;
                robotBodyDef.position = { pos.x, pos.y };
                robotBodyDef.angle = angle;
                robotBodyDef.angularDamping = robotAngularDamping;

                b2Body *body = mWorld->CreateBody(&robotBodyDef);

                // approximate a robot with a cylinder
                const auto r = (double)mProperties.robotRadius;
                const auto h = (double)mProperties.robotHeight;
                const double robotVolume = glm::pi<double>() * r * r * h;
                assert(robotVolume > 0.0);
          
                b2CircleShape shape = {};
                shape.m_radius = (float) r;

                b2FixtureDef fixture = {};
                fixture.density = (float)((double)mProperties.robotMass / robotVolume);
                fixture.friction = mProperties.robotFriction;
                fixture.restitution = mProperties.robotRestitution;
                fixture.shape = &shape;

                body->CreateFixture(&fixture);

                SetFieldFriction(body);

                mRobots[i.id] = body;
            }
        }
    }

    void PhysicsServer::SetFieldFriction(b2Body *target, float maxForceMult, float maxTorqueMult) {
        assert(mWorld);
        assert(mRoomBounds);

        float gravity = 9.8f;
	    float I = target->GetInertia();
	    float mass = target->GetMass();

        // (for notes, refer to apply_force example in box2d testbed)
	    // for a circle: I = 0.5 * m * r * r ==> r = sqrt(2 * I / m)
	    float effectiveRadius = b2Sqrt(2.0f * I / mass);

        maxForceMult *= mProperties.fieldFriction;
        maxTorqueMult *= mProperties.fieldFriction;

        b2FrictionJointDef jd;
	    jd.localAnchorA.SetZero();
	    jd.localAnchorB.SetZero();
	    jd.bodyA = mRoomBounds;
	    jd.bodyB = target;
	    jd.collideConnected = true;
	    jd.maxForce = maxForceMult * mass * gravity;
	    jd.maxTorque = maxTorqueMult * mass * effectiveRadius * gravity;
        
        mWorld->CreateJoint(&jd);
    }

    void PhysicsServer::GameStep(float dt) {
        assert(mWorld);
        assert(mContactListener);

        mContactListener->Clear();

        int32 velocityIterations = 8;
        int32 positionIterations = 3;

        mWorld->Step(dt, velocityIterations, positionIterations);
    }

    void PhysicsServer::UpdateMotorsPower(int robotId, float leftMotorForce, float rightMotorForce) {
        assert(mWorld);
        assert(mRobots.find(robotId) != mRobots.end());

        const float eps = 0.001f;

        b2Body *robot = mRobots[robotId];

        // world-space direction from cos and sin of the angle
        const b2Vec2 dir = { robot->GetTransform().q.c, robot->GetTransform().q.s } ;

        const glm::vec2 motorPositions[] = {
            ToPhysicsCoords({ mProperties.robotLeftMotorOffset.x,  mProperties.robotLeftMotorOffset.y }),
            ToPhysicsCoords({ mProperties.robotRightMotorOffset.x, mProperties.robotRightMotorOffset.y })
        };

        const float forces[] = {
            leftMotorForce,
            rightMotorForce
        };

        for (int i = 0; i < 2; i++) {
            if (std::abs(forces[i]) > eps) {
                const b2Vec2 localMotorPos = { motorPositions[i].x, motorPositions[i].y };
                const b2Vec2 worldMotorPos = robot->GetWorldPoint(localMotorPos);
                const b2Vec2 worldForce = { dir.x * forces[i], dir.y * forces[i] };

                robot->ApplyForce(worldForce, worldMotorPos, true);            
            }
        }
    }

    void PhysicsServer::GetCurrentGameState(PhysicsGameState &state) const {
        assert(mWorld);
        assert(mContactListener);
        
        state.robots.resize(mRobots.size());

        {
            BodyState s = {};
            s.position = ToGameCoords({ mBall->GetPosition().x, mBall->GetPosition().y });
            s.velocity = ToGameCoords({ mBall->GetLinearVelocity().x, mBall->GetLinearVelocity().y });
            s.angle = AngleToGameCoords(mBall->GetAngle());

            state.ball = s;
        }

        for (const auto &r : mRobots) {
            int id = r.first;
            b2Body *body = r.second;

            BodyState s = {};
            s.position = ToGameCoords({ body->GetPosition().x, body->GetPosition().y });
            s.velocity = ToGameCoords({ body->GetLinearVelocity().x, body->GetLinearVelocity().y });
            s.angle = AngleToGameCoords(body->GetAngle());

            state.robots[id] = s;
        }

        std::vector<std::pair<b2Fixture*, b2Fixture*>> cs;
        mContactListener->GetContacts(cs);

        for (const auto &c : cs) {
            b2Body *bodyA = c.first->GetBody();
            b2Body *bodyB = c.second->GetBody();

            int robotA, robotB;
            bool isRobotA = TryGetRobotByBody(bodyA, robotA);
            bool isRobotB = TryGetRobotByBody(bodyB, robotB);

            if (IsBall(bodyA) || IsBall(bodyB)) {
                state.isBallInsideGoal = IsGoal(bodyA) || IsGoal(bodyB);
                state.isBallOutOfFieldBounds =  
                    IsFieldBounds(bodyA) || IsFieldBounds(bodyB) || 
                    IsRoomBounds(bodyA) || IsRoomBounds(bodyB);

                if (isRobotA) {
                    state.robotBallCollisions.push_back(robotA);
                }
                
                if (isRobotB) {
                    state.robotBallCollisions.push_back(robotB);
                }

                continue;
            }

            if (isRobotA && isRobotB) {
                bool wasAdded = false;

                // check if it was already added
                for (const auto &r : state.robotRobotCollisions) {
                    if ((r.robotIdA == robotA && r.robotIdB == robotB) ||
                        (r.robotIdA == robotB && r.robotIdB == robotA)) {
                        wasAdded = true;
                        break;
                    }
                }

                if (wasAdded) {
                    continue;
                }

                RobotCollisionInfo info = {};
                info.robotIdA = robotA;
                info.robotIdB = robotB;

                state.robotRobotCollisions.push_back(info);

                continue;
            }

            if (isRobotB) {
                // let A be a robot
                std::swap(bodyA, bodyB);
                std::swap(robotA, robotB);
                std::swap(isRobotA, isRobotB);
            }

            if (isRobotA) {
                if (IsFieldBounds(bodyB)) {
                    state.robotFieldBoundsCollisions.push_back(robotA);
                }

                if (IsRoomBounds(bodyB)) {
                    state.robotRoomBoundsCollisions.push_back(robotA);
                }
            }
        }
    }

    bool PhysicsServer::IsRoomBounds(b2Body *body) const {
        return body == mRoomBounds;
    }

    bool PhysicsServer::IsFieldBounds(b2Body *body) const {
        return body == mFieldBoundSensors;
    }

    bool PhysicsServer::IsGoal(b2Body *body) const {
        // TODO
        return false;
    }

    bool PhysicsServer::IsBall(b2Body *body) const {
        return body == mBall;
    }

    bool PhysicsServer::TryGetRobotByBody(b2Body *body, int &outId) const {
        for (const auto &r : mRobots) {
            if (r.second == body) {
                outId = r.first;
                return true;
            }
        }

        outId = -1;
        return false;
    }

    void PhysicsServer::EndGame() {
        mContactListener.reset();
        mWorld.reset();

        mRoomBounds = nullptr;
        mFieldBoundSensors = nullptr;
        mRobots.clear();
        mBall = nullptr;
    }

    glm::vec2 PhysicsServer::ToPhysicsCoords(const glm::vec2 &p) {
        return { p.x, -p.y };
    }

    float PhysicsServer::AngleToPhysicsCoords(float angle) {
        // clockwise to counter-clockwise
        return -angle;
    }

    glm::vec2 PhysicsServer::ToGameCoords(const glm::vec2 &p) {
        return { p.x, -p.y };
    }

    float PhysicsServer::AngleToGameCoords(float angle) {
        // counter-clockwise to clockwise
        return -angle;
    }

}
