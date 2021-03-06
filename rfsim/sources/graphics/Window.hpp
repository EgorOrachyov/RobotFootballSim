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

#ifndef RFSIM_WINDOW_HPP
#define RFSIM_WINDOW_HPP

#include <GLFW/glfw3.h>
#include <glm/vec2.hpp>
#include <string>

namespace rfsim {

    /**
     * Wrapper for GLFW window.
     */
    class Window {
    public:
        explicit Window(GLFWwindow* hnd);
        ~Window();

        /** Bind window as drawing target */
        void MakeContextCurrent();
        void SetTitle(const std::string& caption);

        /** @return True if user wants to close window */
        bool ShouldClose() const;

        glm::ivec2 GetPosition() const;
        glm::ivec2 GetSize() const;
        glm::ivec2 GetFramebufferSize() const;

        /** @return Internal glfw hnd */
        GLFWwindow* GetNativeHnd() const;

    private:
        GLFWwindow* mHND = nullptr;
    };

}

#endif //RFSIM_WINDOW_HPP