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
#include <cmath>

namespace rfsim {

    class PhysicsContactListener : public b2ContactListener {
    public:
        explicit PhysicsContactListener(const b2Body *fieldBoundSensors) : mFieldBoundSensors(fieldBoundSensors) {}
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

            mContacts.emplace_back( contact->GetFixtureA(), contact->GetFixtureB() );
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

    PhysicsServer::PhysicsServer(float fixedDt) : 
        mFixedDt(fixedDt), mDtAccumulated(0), 
        mRoomBounds(nullptr), mFieldBoundSensors(nullptr), mBall(nullptr) {
        assert(mFixedDt > 0);
    }

    PhysicsServer::~PhysicsServer() {
        EndGame();
    }

    void PhysicsServer::SetGameProperties(const PhysicsGameProperties &properties) {
        assert(mProperties.robotWheelXOffset >= 0.0f);
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
            // need to calculate area instead of volume for box2d,
            // so the mass can be predicted more accurately
            const double ballArea = glm::pi<double>() * r * r;
            assert(ballArea > 0.0);

            b2CircleShape shape = {};
            shape.m_radius = r;

            b2FixtureDef fixture = {};
            fixture.density = (float)((double)mProperties.ballMass / ballArea);
            fixture.friction = mProperties.ballFriction;
            fixture.restitution = mProperties.ballRestitution;
            fixture.shape = &shape;

            mBall->CreateFixture(&fixture);

            SetBallRollingFriction(mBall);
        }

        // robots
        const std::vector<RobotInitInfo>* allRobots[] = {
            &info.robotsTeamA,
            &info.robotsTeamB
        };

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

                b2Body *body = mWorld->CreateBody(&robotBodyDef);
          
                // approximate a robot with a cylinder
                const float r = mProperties.robotRadius;
                const float h = mProperties.robotHeight;

                b2PolygonShape shape = CreateRobotShape(r);

                // need to calculate area instead of volume for box2d,
                // so the mass can be predicted more accurately;
                // estimate its area 
                const float robotArea = GetRobotArea(shape);
                assert(robotArea > 0.0f);

                b2FixtureDef fixture = {};
                fixture.density = mProperties.robotMass / robotArea;
                fixture.friction = mProperties.robotFriction;
                fixture.restitution = mProperties.robotRestitution;
                fixture.shape = &shape;

                body->CreateFixture(&fixture);

                mRobots[i.id] = body;
            }
        }
    }

    b2PolygonShape PhysicsServer::CreateRobotShape(const double r) {
        const auto pi = glm::pi<double>();

        // Accordingly to the spec:
        // Default robot size is 0.085m
        // Default front offset is 0.050m, so some % here
        const double frontOffsetProportion = 0.050 / 0.085;

        const double radius = r;
        const double frontOffset = r * frontOffsetProportion;

        b2PolygonShape shape = {};
        const int frontPointCount = 2;
        const int circlePointCount = 6;
        const int pointCount = frontPointCount + circlePointCount;

        assert(b2_maxPolygonVertices >= pointCount);
        b2Vec2 points[pointCount];

        double cornerY = std::sqrt(radius * radius - frontOffset * frontOffset);
        double scale = r / radius;

        points[0] = {(float) (frontOffset * scale), (float) (-cornerY * scale)};
        points[1] = {(float)(frontOffset * scale), (float) (cornerY * scale)};

        double initialPointAngle = std::acos(frontOffset / radius);
        double backArc = 2 * pi - initialPointAngle * 2;
        double stepAngle = backArc / circlePointCount; // What are you doing, step-angle?

        for (int i = 0; i < circlePointCount; i += 1) {
            double currentAngle = initialPointAngle + stepAngle * (i + 1);
            double x = r * std::cos(currentAngle);
            double y = r * std::sin(currentAngle);
            points[frontPointCount + i] = { (float) x, (float) y};
        }

        shape.Set(points, pointCount);
        return shape;
    }

    void PhysicsServer::FrameStep(const std::shared_ptr<Game> &game, const std::function<bool(float)> &onFixedStep, float dt) {
        assert(game->physicsGameInitInfo.robotsTeamA.size() >= game->teamSize);
        assert(game->physicsGameInitInfo.robotsTeamB.size() >= game->teamSize);
        assert(game->robotWheelVelocitiesA.size() >= game->teamSize);
        assert(game->robotWheelVelocitiesB.size() >= game->teamSize);

        constexpr int teamCount = 2;

        const std::vector<rfsim::RobotInitInfo> *pRs[teamCount] = {
            &game->physicsGameInitInfo.robotsTeamA,
            &game->physicsGameInitInfo.robotsTeamB
        };

        const std::vector<glm::vec2>* pVs[teamCount] = {
            &game->robotWheelVelocitiesA,
            &game->robotWheelVelocitiesB
        };

        AccumulateDeltaTime(dt);

        while (TryFixedStep() && UpdateState(game) && onFixedStep(GetFixedDt()))
        {
            for (int t = 0; t < teamCount; t++) {
                const auto &rs = *(pRs[t]);
                const auto &vs = *(pVs[t]);

                for (int i = 0; i < game->teamSize; i++) {
                    auto id = rs[i].id;
                    const auto &wv = vs[i];

                    UpdateWheelVelocities(id, wv.x, wv.y);
                }
            }
        }
    }

    void PhysicsServer::SetBallRollingFriction(b2Body *ball, float maxFrictionForceMult, float maxFrictionTorqueMult) {
        assert(mWorld);
        assert(mRoomBounds);

        float gravity = 9.8f;
        float I = ball->GetInertia();
	    float mass = ball->GetMass();

        // rolling resistance coefficient for a golf ball,
        // the value is inspired by "Golf Ball Landing, Bounce and Roll on Turf", Woo-Jin Roha, Chong-Won Leeb
        float Crr = 0.02f;

        // normal force
        float N = mass * gravity;
        // rolling resistance force
        float F = Crr * N;

        // (for notes, refer to apply_force example in box2d testbed)
	    // for a circle: I = 0.5 * m * r * r ==> r = sqrt(2 * I / m)
	    float effectiveRadius = b2Sqrt(2.0f * I / mass);
        // coarse fix for the fact that ball touches a field with some small area
        effectiveRadius *= 0.05f;

        b2FrictionJointDef jd;
	    jd.localAnchorA.SetZero();
	    jd.localAnchorB.SetZero();
	    jd.bodyA = mRoomBounds;
	    jd.bodyB = ball;
	    jd.collideConnected = true;
	    jd.maxForce = maxFrictionForceMult * F;
	    jd.maxTorque = maxFrictionTorqueMult * mass * effectiveRadius * gravity;
        
        mWorld->CreateJoint(&jd);
    }

    void PhysicsServer::AccumulateDeltaTime(float dt) {
        mDtAccumulated += dt;
    }

    float PhysicsServer::GetFixedDt() const
    {
        return mFixedDt;
    }

    bool PhysicsServer::TryFixedStep() {
        assert(mWorld);
        assert(mContactListener);

        if (mDtAccumulated < mFixedDt) {
            return false;
        }

        mContactListener->Clear();

        int32 velocityIterations = 8;
        int32 positionIterations = 3;

        float maxSpeed = mProperties.robotMaxSpeed;

        for (const auto &r : mRobots) {
            if (r.second->GetLinearVelocity().LengthSquared() > maxSpeed * maxSpeed) {
                b2Vec2 v = r.second->GetLinearVelocity();
                v.Normalize();
                r.second->SetLinearVelocity(maxSpeed * v);
            }
        }

        mWorld->Step(mFixedDt, velocityIterations, positionIterations);

        mDtAccumulated -= mFixedDt;

        return true;
    }

    bool PhysicsServer::UpdateState(const std::shared_ptr<Game> &game) {
        GetCurrentGameState(game->physicsGameState);
        return true;
    }

    void PhysicsServer::UpdateWheelVelocities(int robotId, float leftWheelVelocity, float rightWheelVelocity) {
        assert(mWorld);
        assert(mRobots.find(robotId) != mRobots.end());

        const float eps = 0.001f;

        if (std::abs(leftWheelVelocity) < eps && std::abs(rightWheelVelocity) < eps) {
            return;
        }

        b2Body *robot = mRobots[robotId];
        
        const float dt = mFixedDt;

        // Differential Drive Robots
        // http://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf

        float x = robot->GetTransform().p.x;
        float y = robot->GetTransform().p.y;

        float theta = robot->GetAngle();
        float cos_theta = robot->GetTransform().q.c;
        float sin_theta = robot->GetTransform().q.s;

        float Vl = leftWheelVelocity;
        float Vr = rightWheelVelocity;

        float l = mProperties.robotWheelXOffset * 2;

        // if R is infinite (so no rotation)
        // or no distance between wheels
        if (std::abs(Vl - Vr) < eps || l < eps) {
            b2Vec2 dir = { cos_theta, sin_theta };
            float v = (Vr + Vl) / 2.0f;

            robot->SetLinearVelocity(v * dir);
            robot->SetAngularVelocity(0);
        } else {
            float R = l / 2.0f * (Vl + Vr) / (Vr - Vl);
            float w = (Vr - Vl) / l;

            // Instantaneous Center of Curvature
            b2Vec2 ICC = { x - R * sin_theta, y + R * cos_theta };

            // move ICC to origin
            const float x_i = x - ICC.x;
            const float y_i = y - ICC.y;

            // rotate by w in ICC space
            float x_n = cosf(w * dt) * x_i - sinf(w * dt) * y_i;
            float y_n = sinf(w * dt) * x_i + cosf(w * dt) * y_i;

            // move from ICC space to world space
            x_n += ICC.x;
            y_n += ICC.y;
            
            robot->SetLinearVelocity(1.0f / dt * b2Vec2{ x_n - x, y_n - y });
            robot->SetAngularVelocity(w);
        }
    }

    void PhysicsServer::GetCurrentGameState(PhysicsGameState &state) const {
        assert(mWorld);
        assert(mContactListener);

        state.robots.resize(mRobots.size());

        state.robotRobotCollisions.clear();
        state.robotFieldBoundsCollisions.clear();
        state.robotRoomBoundsCollisions.clear();
        state.robotBallCollisions.clear();

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

    float PhysicsServer::GetRobotArea(const b2PolygonShape &shape)
    {
        // from b2PolygonShape::ComputeMass()
        float area = 0.0f;

        b2Vec2 s = shape.m_vertices[0];

	    for (int32 i = 0; i < shape.m_count; ++i)
	    {
		    // Triangle vertices.
		    b2Vec2 e1 = shape.m_vertices[i] - s;
		    b2Vec2 e2 = i + 1 < shape.m_count ? shape.m_vertices[i+1] - s : shape.m_vertices[0] - s;

		    float D = b2Cross(e1, e2);

		    float triangleArea = 0.5f * D;
		    area += triangleArea;
        }

        return area;
    }
}
