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

#include <graphics/Window.hpp>
#include <cassert>

namespace rfsim {

    Window::Window(struct GLFWwindow *hnd) {
        assert(hnd);
        mHND = hnd;
    }

    Window::~Window() {
        assert(mHND);
        mHND = nullptr;
    }

    void Window::MakeContextCurrent() {
        glfwMakeContextCurrent(mHND);
    }

    void Window::SetTitle(const std::string &caption) {
        glfwSetWindowTitle(mHND, caption.c_str());
    }

    bool Window::ShouldClose() const {
        return glfwWindowShouldClose(mHND);
    }

    glm::ivec2 Window::GetPosition() const {
        glm::ivec2 position;
        glfwGetWindowPos(mHND, &position.x, &position.y);
        return position;
    }

    glm::ivec2 Window::GetSize() const {
        glm::ivec2 size;
        glfwGetWindowSize(mHND, &size.x, &size.y);
        return size;
    }

    glm::ivec2 Window::GetFramebufferSize() const {
        glm::ivec2 fboSize;
        glfwGetFramebufferSize(mHND, &fboSize.x, &fboSize.y);
        return fboSize;
    }

    GLFWwindow * Window::GetNativeHnd() const {
        return mHND;
    }


}