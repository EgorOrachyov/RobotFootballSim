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

#include <gui/GuiMainMenu.hpp>
#include <gui/GuiApplication.hpp>

namespace rfsim {

    GuiMainMenu::GuiMainMenu(class GuiApplication &app) : mApp(app) {

    }

    void GuiMainMenu::Refresh() {
        mAlgorithms.emplace_back("algo 1");
        mAlgorithms.emplace_back("algo 2");
        mAlgorithms.emplace_back("algo 3");
        mAlgorithms.emplace_back("algo 4");
        mScenarios.emplace_back("scenario 1");
        mScenarios.emplace_back("scenario 2");
        mScenarios.emplace_back("scenario 3");
        mScenarios.emplace_back("scenario 4");
        mScenarios.emplace_back("scenario 5");
        mScenarios.emplace_back("scenario 6");

        mScenariosRaw.clear();
        mScenariosRaw.reserve(mScenarios.size());

        for (auto& s: mScenarios) {
            mScenariosRaw.push_back(s.data());
        }

        mAlgorithmsRaw.clear();
        mAlgorithmsRaw.reserve(mAlgorithms.size());

        for (auto& a: mAlgorithms) {
            mAlgorithmsRaw.push_back(a.data());
        }
    }

    void GuiMainMenu::Restore() {
        // do nothing
    }

    void GuiMainMenu::Update() {
        auto& style = mApp.GetStyle();

        ImGui::Begin("Main Menu");

        ImGui::Text("How to start a new game:");
        ImGui::Text(" - 1. Select the game scenario");
        ImGui::Text(" - 2. Select the algorithm used to control robots");
        ImGui::Text(" - 3. Press start button");
        ImGui::NewLine();

        ImGui::ListBox("Game scenario", &mSelectedScenario, mScenariosRaw.data(), mScenariosRaw.size());
        ImGui::ListBox("Algorithm", &mSelectedAlgo, mAlgorithmsRaw.data(), mAlgorithmsRaw.size());

        ImGui::PushStyleColor(ImGuiCol_Button, style.greenColor);
        mBeginGame = ImGui::Button("Start", ImVec2(200, 0));

        ImGui::SameLine();

        ImGui::PushStyleColor(ImGuiCol_Button, style.redColor);
        mExit = ImGui::Button("Exit", ImVec2(200, 0));

        ImGui::PopStyleColor(2);

        ImGui::NewLine();
        ImGui::Text("Robot Football Simulator (c)");

        ImGui::End();
    }

    void GuiMainMenu::Hide() {
        // do nothing
    }

    bool GuiMainMenu::BeginGame() const {
        return mBeginGame;
    }

    bool GuiMainMenu::Exit() const {
        return mExit;
    }

    int GuiMainMenu::GetSelectedScenarioId() const {
        return mSelectedScenario;
    }

    int GuiMainMenu::GetSelectedAlgorithmId() const {
        return mSelectedAlgo;
    }

}