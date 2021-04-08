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

#ifndef RFSIM_GRAPHICSSETTINGS_HPP
#define RFSIM_GRAPHICSSETTINGS_HPP

#include <glm/vec3.hpp>

namespace rfsim {

    /**
     * @brief Game draw settings.
     * Configures effects and level of drawing game details.
     */
    struct GraphicsSettings {
        bool drawTrace = false;
        int traceLength = 20;
        float traceSkip = 0.2;
        float tracePointRadius = 0.3;

        bool drawRobotIDs = true;
        bool drawOutInfo = true;
        bool drawCollisionInfo = true;

        bool drawShadows = true;
        float shadowIntensity = 1.0f;
        float sunPosition = 0.0f;

        bool drawMarkers = true;
        float markerProp = 1.2f;
        float markerOpacity = 0.8f;
        glm::vec3 team1Color = {0.95f, 0.05f, 0.05f };
        glm::vec3 team2Color = {0.05f, 0.05f, 0.95f };

        glm::vec3 traceColor = {1.0f, 1.0f, 1.0f };
        glm::vec3 backgroundColor = {0, 0, 0};
        glm::vec3 fieldCustomColor = { 1.0f, 1.0f, 1.0f };
    };

}

#endif //RFSIM_GRAPHICSSETTINGS_HPP