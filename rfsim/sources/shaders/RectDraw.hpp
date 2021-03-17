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

#ifndef RFSIM_RECTDRAW_HPP
#define RFSIM_RECTDRAW_HPP

#include <string>

namespace rfsim {

    std::string RECT_DRAW_VERTEX_SOURCE = R"(
    // Remember: macOS max supported version is 4.1
    #version 410 core

    layout (location = 0) in vec3 position;
    layout (location = 1) in vec4 penColor;
    layout (location = 2) in vec4 brushColor;
    layout (location = 3) in float isFilled;

    uniform vec2 areaSize;
    uniform mat4 projView;

    flat out vec4 fsPenColor;
    flat out vec4 fsBrushColor;
    flat out int fsIsFilled;

    void main() {
        fsPenColor = penColor;
        fsBrushColor = brushColor;
        fsIsFilled = isFilled != 0.0f ? 1: 0;
        gl_Position = projView * vec4(position.x, areaSize.y - position.y, position.z, 1.0f);
    }
    )";

    std::string RECT_DRAW_FRAGMENT_SOURCE = R"(
    // Remember: macOS max supported version is 4.1
    #version 410 core

    layout (location = 0) out vec4 outColor;

    flat in vec4 fsPenColor;
    flat in vec4 fsBrushColor;
    flat in int fsIsFilled;

    #define COLOR_GAMMA 2.2

    vec4 convertSRBGtoLinear(in vec4 color) {
        float alpha = color.a;
        vec3 converted = pow(color.rgb,vec3(COLOR_GAMMA));
        return vec4(converted,alpha);
    }

    vec4 convertLinearToSRGB(in vec4 color) {
        float alpha = color.a;
        vec3 converted = pow(color.rgb,vec3(1.0f / COLOR_GAMMA));
        return vec4(converted,alpha);
    }

    void main() {
        vec4 color;

        if (fsIsFilled != 0)
            color = fsBrushColor;
        else
            color = fsPenColor;

        outColor = convertLinearToSRGB(color);
        outColor.a = clamp(outColor.a, 0.0f, 1.0f);
    }
    )";

}

#endif //RFSIM_RECTDRAW_HPP
