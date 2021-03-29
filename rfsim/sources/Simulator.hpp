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

#ifndef RFSIM_SIMULATOR_HPP
#define RFSIM_SIMULATOR_HPP

#include <memory>
#include <vector>
#include <string>

namespace rfsim {

    /**
     * @brief Simulator main class.
     *
     * Manages sub-systems, update loop, application start-up and configuration parsing.
     */
    class Simulator {
    public:
        /**
         * Create the simulator class.
         *
         * @param argc Number of the OS native app args
         * @param argv AActual arguments
         */
        Simulator(int argc, const char* const* argv);
        virtual ~Simulator();

        /**
         * Run the main simulator update loop.
         * This function returns control only when user closes the application.
         *
         * @return 0 if simulator successfully finished.
         */
        virtual int Run();

    protected:
        int mWindowWidth = 1920;
        int mWindowHeight = 1280;
        std::string mResourcesPath = "../resources";
        std::string mPluginsPath = ".";
        std::vector<std::string> mArgs;

        std::shared_ptr<class Window> mPrimaryWindow;
        std::shared_ptr<class WindowManager> mWindowManager;
        std::shared_ptr<class Painter> mPainter;
        std::shared_ptr<class GraphicsServer> mGraphicsServer;
        std::shared_ptr<class PhysicsServer> mPhysicsServer;
        std::shared_ptr<class AlgorithmManager> mAlgorithmManager;
        std::shared_ptr<class GameManager> mGameManager;
    };

}

#endif //RFSIM_SIMULATOR_HPP