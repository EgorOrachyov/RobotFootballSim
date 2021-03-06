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

#ifndef RFSIM_PHYSICSGAMEPROPERTIES_HPP
#define RFSIM_PHYSICSGAMEPROPERTIES_HPP

#include "glm/vec2.hpp"

namespace rfsim {

    /**
     * @brief General game physics settings.
     *
     * Robots size, mass, friction,
     * ball restitution, field friction.
     */
    struct PhysicsGameProperties {
        float fieldFriction = 0.0f;

        float robotRadius = 0.0f;
        float robotHeight = 0.0f;
        float robotMass = 0.0f;
        float robotFriction = 0.0f;
        float robotRestitution = 0.0f;
        float robotMaxSpeed = 0.0f;
        // Offset for left and right wheels from the center.
        float robotWheelXOffset = 0.0f;

        float ballRadius = 0.0f;
        float ballMass = 0.0f;
        float ballFriction = 0.0f;
        float ballRestitution = 0.0f;
    };

}

#endif //RFSIM_PHYSICSGAMEPROPERTIES_HPP
