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

#ifndef RFSIM_GUIMAINMENU_HPP
#define RFSIM_GUIMAINMENU_HPP

#include <gui/GuiWidget.hpp>
#include <vector>
#include <string>

namespace rfsim {

    class GuiMainMenu: public GuiWidget {
    public:
        explicit GuiMainMenu(class GuiApplication& app);
        ~GuiMainMenu() override = default;

        void Refresh() override;
        void Restore() override;
        void Update() override;
        void Hide() override;

        bool BeginGame() const;
        bool Exit() const;
        int GetSelectedScenarioId() const;
        int GetSelectedAlgorithmId() const;

    private:
        bool mBeginGame = false;
        bool mExit = false;

        int mSelectedScenario = -1;
        int mSelectedAlgo = -1;

        std::vector<std::string> mScenarios;
        std::vector<const char*> mScenariosRaw;
        std::vector<std::string> mAlgorithms;
        std::vector<const char*> mAlgorithmsRaw;

        class GuiApplication& mApp;
    };

}

#endif //RFSIM_GUIMAINMENU_HPP