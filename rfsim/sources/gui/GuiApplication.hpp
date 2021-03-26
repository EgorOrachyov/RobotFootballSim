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

#ifndef RFSIM_GUIAPPLICATION_HPP
#define RFSIM_GUIAPPLICATION_HPP

#include <graphics/Window.hpp>
#include <graphics/WindowManager.hpp>
#include <graphics/Painter.hpp>
#include <graphics/GraphicsServer.hpp>
#include <graphics/Image.hpp>
#include <physics/PhysicsServer.hpp>
#include <logic/AlgorithmManager.hpp>
#include <gui/GuiInGameMenu.hpp>
#include <gui/GuiMainMenu.hpp>
#include <gui/GuiStyle.hpp>
#include <memory>

namespace rfsim {

    class GuiApplication {
    public:
        GuiApplication(const std::shared_ptr<Window> &window,
                       const std::shared_ptr<WindowManager> &winMan,
                       const std::shared_ptr<Painter> &painter,
                       const std::shared_ptr<PhysicsServer> &physics,
                       const std::shared_ptr<GraphicsServer> &graphics,
                       const std::shared_ptr<AlgorithmManager> &algoMan,
                       const std::string& resPath);

        ~GuiApplication() = default;

        int Run();
        GuiStyle& GetStyle();

    private:
        friend class GuiMainMenu;
        friend class GuiInGameMenu;

        enum class GlobalState {
            MainMenu,
            PrepareGame,
            InGame,
        };

        float mFontScale = 2.0f;
        float mGuiScale = 2.0f;

        GlobalState mState = GlobalState::MainMenu;
        GuiStyle mStyle;

        std::string mResPath;

        std::shared_ptr<Window> mWindow;
        std::shared_ptr<WindowManager> mWindowManager;
        std::shared_ptr<Painter> mPainter;
        std::shared_ptr<GraphicsServer> mGraphicsServer;
        std::shared_ptr<PhysicsServer> mPhysicsServer;
        std::shared_ptr<AlgorithmManager> mAlgorithmManager;

        std::shared_ptr<GuiMainMenu> mMainMenu;
        std::shared_ptr<GuiInGameMenu> mInGameMenu;

        std::shared_ptr<Image> mMainMenuLogo;
        std::shared_ptr<Image> mMainMenuBall;
    };

}

#endif //RFSIM_GUIAPPLICATION_HPP