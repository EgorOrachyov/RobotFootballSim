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

#include <graphics/WindowManager.hpp>
#include <iostream>

namespace rfsim {

    WindowManager::WindowManager() {
        glfwInit();
        glfwSetErrorCallback(ErrorCallback);

#ifdef RFSIM_PLATFORM_MACOS
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // Required on Mac
#else
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif
    }

    WindowManager::~WindowManager() {
        for (auto& window: mWindows) {
            assert(window.unique());

            GLFWwindow* hnd = window->GetNativeHnd();
            glfwDestroyWindow(hnd);
        }

        mWindows.clear();
    }

    std::shared_ptr<class Window> WindowManager::CreateWindow(const glm::ivec2 &size, const std::string &title) {
        GLFWwindow* hnd = glfwCreateWindow(size.x, size.y, title.c_str(), nullptr, nullptr);

        auto window = std::make_shared<Window>(hnd);
        window->MakeContextCurrent();
        mWindows.push_back(window);

        return window;
    }

    void WindowManager::UpdateFrame() {
        // OS events
        glfwPollEvents();

        // Swap buffers to display GL drawn logic
        for (auto& window: mWindows) {
            GLFWwindow* hnd = window->GetNativeHnd();
            glfwSwapBuffers(hnd);
        }
    }

    void WindowManager::ErrorCallback(int errorCode, const char *description) {
#ifdef RFSIM_DEBUG
        std::cerr << "GLFW error: code=" << errorCode << ", desc=\"" << description << "\"" << std::endl;
#endif
    }

}