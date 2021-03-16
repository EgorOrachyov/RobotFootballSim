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

#include <opengl/GLShader.hpp>
#include <iostream>
#include <exception>
#include <cassert>

namespace rfsim {

    GLShader::GLShader(const std::vector<char> &vertexStage, const std::vector<char> &fragmentStage) {
        mNumStages = 2;

        GLenum types[] = {GL_VERTEX_SHADER, GL_FRAGMENT_SHADER};
        size_t length[] = { vertexStage.size(), fragmentStage.size() };
        const char* source[] = { vertexStage.data(), fragmentStage.data() };

        bool hasError = false;

        for (size_t i = 0; i < mNumStages; i++) {
            GLuint handle = glCreateShader(types[i]);

            const char* sources[] = { source[i] };
            GLint lengths[] = { (int) length[i] };

            glShaderSource(handle, 1, sources, lengths);
            glCompileShader(handle);

            int result = 0;
            glGetShaderiv(handle, GL_COMPILE_STATUS, &result);

            if (!result) {
                int logLength;
                glGetShaderiv(handle, GL_INFO_LOG_LENGTH, &logLength);

                if (logLength > 0) {
                    std::vector<char> log;
                    log.resize(logLength + 1);

                    int written;
                    glGetShaderInfoLog(handle, logLength, &written, log.data());
                    log[logLength] = '\0';

                    std::cerr << "Failed compile shader: " << log.data() << std::endl;
                }

                glDeleteShader(handle);
                hasError = true;
            }
            else {
                mStages[i] = handle;
            }
        }

        if (hasError) {
            throw std::runtime_error("Failed to compile shader stages");
        }

        mProgram = glCreateProgram();

        for (size_t i = 0; i < mNumStages; i++) {
            glAttachShader(mProgram, mStages[i]);
        }

        glLinkProgram(mProgram);

        int status;
        glGetProgramiv(mProgram, GL_LINK_STATUS, &status);

        if (!status) {
            int logLength;
            glGetProgramiv(mProgram, GL_INFO_LOG_LENGTH, &logLength);

            if (logLength > 0) {
                std::vector<char> log;
                log.resize(logLength + 1);

                int written;
                glGetShaderInfoLog(mProgram, logLength, &written, log.data());
                log[logLength] = '\0';

                std::cerr << "Failed link shader program: " << log.data() << std::endl;
            }

            throw std::runtime_error("Failed to link shader program");
        }
    }

    GLShader::~GLShader() {
        if (mProgram) {
            glDeleteProgram(mProgram);

            for (size_t i = 0; i < mNumStages; i++) {
                glDeleteShader(mStages[i]);
            }

            mProgram = 0;
        }
    }

    GLuint GLShader::GetProgram() const {
        return mProgram;
    }

    void GLShader::Bind() const {
        assert(mProgram);
        glUseProgram(mProgram);
    }

    void GLShader::Unbind() const {
        glUseProgram(0);
    }

    void GLShader::SetVec2(const std::string &name, const glm::vec2 &vec) const {
        int location = GetLocation(name);
        glUniform2f(location, vec.x, vec.y);
    }

    void GLShader::SetMatrix4(const std::string &name, const glm::mat4 &mat) const {
        int location = GetLocation(name);
        glUniformMatrix4fv(location, 1, GL_FALSE, (const float*) &mat[0][0]);
    }

    void GLShader::SetTexture(const std::string &name, const std::shared_ptr<GLTexture> &texture, int slot) const {
        int location = GetLocation(name);
        glUniform1i(location, slot);
        glActiveTexture(GL_TEXTURE0 + slot);
        glBindTexture(GL_TEXTURE_2D, texture->GetTextureHnd());
    }

    int GLShader::GetLocation(const std::string &name) const {
        int location = glGetUniformLocation(mProgram, name.c_str());

        if (location < 0) {
            std::cerr << "No such uniform variable in shader program:" << name << std::endl;
            throw std::runtime_error("Failed to find uniform variable location");
        }

        return location;
    }

}

