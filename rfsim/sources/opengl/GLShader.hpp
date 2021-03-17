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

#ifndef RFSIM_GLSHADER_HPP
#define RFSIM_GLSHADER_HPP

#include <GL/glew.h>
#include <glm/matrix.hpp>
#include <opengl/GLTexture.hpp>
#include <vector>
#include <array>
#include <string>

namespace rfsim {

    /**
     * Wrapper for OpenGL program.
     * Allows to create from sources, compile and link complete OpenGL program.
     */
    class GLShader {
    public:
        static const size_t MAX_SHADER_STAGES = 5;

        GLShader(const std::vector<char>& vertexStage, const std::vector<char> &fragmentStage);
        ~GLShader();

        GLuint GetProgram() const;

        void Bind() const;
        void Unbind() const;

        void SetBool(const std::string& name, bool value) const;
        void SetVec2(const std::string& name, const glm::vec2& vec) const;
        void SetVec4(const std::string& name, const glm::vec4& vec) const;
        void SetMatrix4(const std::string& name, const glm::mat4& mat) const;
        void SetTexture(const std::string& name, const std::shared_ptr<GLTexture> &texture, int slot) const;

    private:
        int GetLocation(const std::string& name) const;

        size_t mNumStages = 0;
        GLuint mProgram = 0;
        std::array<GLuint, MAX_SHADER_STAGES> mStages{};
    };

}

#endif //RFSIM_GLSHADER_HPP