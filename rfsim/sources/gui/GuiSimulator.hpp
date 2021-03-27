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

#ifndef RFSIM_GUISIMULATOR_HPP
#define RFSIM_GUISIMULATOR_HPP

#include <gui/GuiStyle.hpp>
#include <Simulator.hpp>
#include <memory>

namespace rfsim {

    /**
     * @brief Simulator with GUI
     *
     * This is an extension to base simulator with graphical user interface based on imgui library.
     * This is a solid class, which encapsulates all drawn GUI elements for simplicity.
     */
    class GuiSimulator: public Simulator {
    public:
        /**
         * Setup simulator class.
         *
         * @param argc Number of command line arguments
         * @param argv Array of arguments
         */
        GuiSimulator(int argc, const char* const* argv);
        ~GuiSimulator() override = default;

        /**
         * Run simulator application.
         * This function returns when user closes simulator application.
         *
         * @return Status code
         */
        int Run() override;

    private:
        friend class GuiMainMenu;
        friend class GuiInGameMenu;

        enum class State {
            MainMenu,
            PrepareGame,
            InGame,
        };

        // Scale options for hdpi displays
        float mFontScale = 2.0f;
        float mGuiScale = 2.0f;

        GuiStyle mStyle;
        State mState = State::MainMenu;

        // For main menu animated logo
        std::shared_ptr<class Image> mMainMenuLogo;
        std::shared_ptr<class Image> mMainMenuBall;
    };

}

#endif //RFSIM_GUISIMULATOR_HPP