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

#include <gui/GuiMenuBar.hpp>
#include <imgui.h>

namespace rfsim {

    void GuiMenuBar::Update() {
        if (ImGui::BeginMainMenuBar()) {
            if (ImGui::BeginMenu("General")) {
                mShowGraphicsSettings |= ImGui::MenuItem("Settings");
                quit |= ImGui::MenuItem("Quit");
                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Help")) {
                mShowAbout |= ImGui::MenuItem("About");
                ImGui::EndMenu();
            }

            ImGui::EndMainMenuBar();
        }

        if (mShowGraphicsSettings)
            ImGui::OpenPopup("Settings");
        if (mShowAbout)
            ImGui::OpenPopup("About");

        // Popup graphics settings
        if (ImGui::BeginPopup("Settings")) {
            ImGui::SliderFloat("Sim Time Scale", &timeScale, 0.1f, 4.0f);
            ImGui::Separator();

            ImGui::Checkbox("Display Debug Info", &showDebugRobotInfo);
            ImGui::Separator();

            ImGui::Checkbox("Display Trace", &graphicsSettings.drawTrace);
            if (graphicsSettings.drawTrace) {
                ImGui::SliderInt("Trace Length", &graphicsSettings.traceLength, 5, 50);
                ImGui::SliderFloat("Trace Sampling Period", &graphicsSettings.traceSkip, 0.01f, 1.0f);
                ImGui::SliderFloat("Trace Point Size", &graphicsSettings.tracePointRadius, 0.0f, 1.0f);
                ImGui::ColorEdit3("Trace Point Color", &graphicsSettings.traceColor[0]);
            }
            ImGui::Separator();

            ImGui::Checkbox("Display Collision Info", &graphicsSettings.drawCollisionInfo);
            ImGui::Checkbox("Display Out Info", &graphicsSettings.drawOutInfo);
            ImGui::Separator();

            ImGui::Checkbox("Display Shadows", &graphicsSettings.drawShadows);
            if (graphicsSettings.drawShadows) {
                ImGui::SliderFloat("Shadow Intensity", &graphicsSettings.shadowIntensity, 0.0f, 1.0f);
                ImGui::SliderFloat("Sun Position", &graphicsSettings.sunPosition, -0.2f, 0.2f);
            }
            ImGui::Separator();

            ImGui::ColorEdit3("Field Color", &graphicsSettings.fieldCustomColor[0]);

            if (ImGui::Button("Close")) {
                mShowGraphicsSettings = false;
                ImGui::CloseCurrentPopup();
            }

            ImGui::EndPopup();
        }

        // Popup with about editor info
        if (ImGui::BeginPopup("About")) {
            ImGui::Text("Robot Football Sim 1.0.0");
            ImGui::NewLine();
            ImGui::Text("Robot football simulator with graphical user interface ");
            ImGui::Text("and the feature to develop and run algorithms, required ");
            ImGui::Text("to control robots, test various game strategies and game mechanics.");
            ImGui::NewLine();
            ImGui::Text("Copyright (c) 2021 The RobotFootballSim project authors ");

            if (ImGui::Button("Close")) {
                mShowAbout = false;
                ImGui::CloseCurrentPopup();
            }

            ImGui::EndPopup();
        }
    }

}
