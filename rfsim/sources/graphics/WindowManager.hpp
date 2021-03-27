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

#ifndef RFSIM_WINDOWMANAGER_HPP
#define RFSIM_WINDOWMANAGER_HPP

#include <graphics/Window.hpp>
#include <memory>
#include <vector>

namespace rfsim {

    /**
     * Wrapper for GLFW windowing logic.
     */
    class WindowManager {
    public:
        WindowManager();
        ~WindowManager();

        /**
         * Creates native OS window with specified properties.
         *
         * @param size Window size in abstract units
         * @param title Window title displayed to the user
         *
         * @return Window object
         */
        std::shared_ptr<class Window> CreateNewWindow(const glm::ivec2& size, const std::string& title);

        /**
         * Updates windowing system.
         * Queries user input, updates elements states.
         *
         * @note Must be called every frame for smooth update.
         */
        void Update();

    private:

        /** Error callback for glfw */
        static void ErrorCallback(int errorCode, const char *description);

        std::vector<std::shared_ptr<class Window>> mWindows;
    };

}

#endif //RFSIM_WINDOWMANAGER_HPP