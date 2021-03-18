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

#include <box2d/box2d.h>
#include <glm/ext/scalar_constants.hpp>

namespace rfsim {

PhysicsServer::PhysicsServer()
{
}

PhysicsServer::~PhysicsServer()
{
    mWorld.reset();
}

void PhysicsServer::SetGameProperties(const PhysicsGameProperties &properties)
{
    mProperties = properties;
}

void PhysicsServer::BeginGame(const PhysicsGameInitInfo &info)
{
    assert(info.robotsTeamA.size() == info.robotsTeamB.size());
    assert(!mWorld);

    // TODO: fieldFriction using friction joints

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

        b2EdgeShape shape;

        b2FixtureDef commonFixture = {};
        commonFixture.shape = &shape;
        commonFixture.density = 0.0f;
        commonFixture.friction = roomFriction;
        commonFixture.restitution = roomRestitution;

        // left vertical
        shape.SetTwoSided(b2Vec2(fL, fT), b2Vec2(fL, fB));
		mRoomBounds->CreateFixture(&commonFixture);

        // right vertical
        shape.SetTwoSided(b2Vec2(fR, fT), b2Vec2(fR, fB));
		mRoomBounds->CreateFixture(&commonFixture);

        // top horizontal
        shape.SetTwoSided(b2Vec2(fL, fT), b2Vec2(fR, fT));
		mRoomBounds->CreateFixture(&commonFixture);

        // bottom horizontal
        shape.SetTwoSided(b2Vec2(fL, fB), b2Vec2(fR, fB));
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
        
        b2PolygonShape shape;

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

    // ball
    {
        b2BodyDef dynamicBodyDef = {};
        dynamicBodyDef.type = b2_dynamicBody;
        dynamicBodyDef.position = { ballPosition.x, ballPosition.y };

        mBall = mWorld->CreateBody(&dynamicBodyDef);

        const double r = (double)mProperties.ballRadius;
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
    }

    // robots
    const std::vector<PhysicsGameInitInfo::RobotInitInfo>* allRobots[] = {
        &info.robotsTeamA,
        &info.robotsTeamB
    };

    for (const auto *lst : allRobots)
    {
        for (const auto &i : *lst)
        {
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
            const double r = (double)mProperties.robotRadius;
            const double h = (double)mProperties.robotHeight;
            const double robotVolume = glm::pi<double>() * r * r * h;
            assert(robotVolume > 0.0);
      
            b2CircleShape shape = {};
            shape.m_radius = r;

            b2FixtureDef fixture = {};
            fixture.density = (float)((double)mProperties.robotMass / robotVolume);
            fixture.friction = mProperties.robotFriction;
            fixture.restitution = mProperties.robotRestitution;
            fixture.shape = &shape;

            body->CreateFixture(&fixture);

            // TODO: remove
            float rx = rand();
            float ry = rand();

            rx /= RAND_MAX;
            ry /= RAND_MAX;

            rx = (rx - 0.5f) * 2;
            ry = (ry - 0.5f) * 2;

            body->ApplyForce({rx * 30000, ry * 30000}, body->GetPosition(), true);

            mRobots[i.id] = body;
        }
    }

    // robot IDs must be less than robot count
    for (const auto &r : mRobots)
    {
        assert(r.first < (int)mRobots.size());
    }
}

void PhysicsServer::GameStep(float dt)
{
    assert(mWorld);

    int32 velocityIterations = 8;
    int32 positionIterations = 3;

    mWorld->Step(dt, velocityIterations, positionIterations);

    // TODO: physics world step
}

void PhysicsServer::UpdateMotorsPower(int robotId, float leftMotorForce, float rightMotorForce)
{
    assert(mWorld);
    assert(mRobots.find(robotId) != mRobots.end());

    const float eps = 0.001f;

    b2Body *robot = mRobots[robotId];

    // world-space direction from cos and sin of the angle
    const b2Vec2 dir = { robot->GetTransform().q.c, robot->GetTransform().q.s } ;

    const b2Vec2 localMotorPos[] = {
        { mProperties.robotLeftMotorOffset.x,  mProperties.robotLeftMotorOffset.y },
        { mProperties.robotRightMotorOffset.x, mProperties.robotRightMotorOffset.y }
    };

    const float forces[] = {
        leftMotorForce,
        rightMotorForce
    };

    for (int i = 0; i < 2; i++)
    {
        if (forces[i] < eps)
        {
            const b2Vec2 worldMotorPos = robot->GetWorldPoint(localMotorPos[i]);
            const b2Vec2 worldForce = { dir.x * forces[i], dir.y * forces[i] };

            robot->ApplyForce(worldForce, worldMotorPos, true);            
        }
    }
}

void PhysicsServer::GetCurrentGameState(PhysicsGameState &state) const
{
    assert(mWorld);
    
    state.robots.resize(mRobots.size());

    {
        PhysicsGameState::BodyState s = {};
        s.position = ToGameCoords({ mBall->GetPosition().x, mBall->GetPosition().y });
        s.velocity = ToGameCoords({ mBall->GetLinearVelocity().x, mBall->GetLinearVelocity().y });
        s.angle = AngleToGameCoords(mBall->GetAngle());

        state.ball = s;
    }

    for (const auto &r : mRobots)
    {
        int id = r.first;
        b2Body *body = r.second;

        PhysicsGameState::BodyState s = {};
        s.position = ToGameCoords({ body->GetPosition().x, body->GetPosition().y });
        s.velocity = ToGameCoords({ body->GetLinearVelocity().x, body->GetLinearVelocity().y });
        s.angle = AngleToGameCoords(body->GetAngle());

        state.robots[id] = s;
    }

    // TODO: collisions info
    //state.collisions. ;
}

void PhysicsServer::EndGame()
{
    mWorld.reset();

    mRoomBounds = nullptr;
    mFieldBoundSensors = nullptr;
    mRobots.clear();
    mBall = nullptr;
}

glm::vec2 PhysicsServer::ToPhysicsCoords(const glm::vec2 &p)
{
    return { p.x, -p.y };
}

float PhysicsServer::AngleToPhysicsCoords(float angle)
{
    // clockwise to counter-clockwise
    return -angle;
}

glm::vec2 PhysicsServer::ToGameCoords(const glm::vec2 &p)
{
    return { p.x, -p.y };
}

float PhysicsServer::AngleToGameCoords(float angle)
{
    // counter-clockwise to clockwise
    return -angle;
}
}
