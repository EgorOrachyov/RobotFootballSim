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

#include "ConfigManager.hpp"

#include <fstream>
#include "picojson/picojson.h"

namespace rfsim {

    ConfigManager::ConfigManager(const std::string &configFilePath) :
        mWindowWidth(1920), mWindowHeight(1280), mFontScale(1.0f), mGuiScale(1.0f), mResourcesPath("./resources"), mPluginPathPrefix(".") {

        std::ifstream configFile;
        if (!TryToFindConfig(configFilePath, configFile)) {
            std::cout << "Can't find config file: " << configFilePath << std::endl;
        }

        picojson::value result;
        const std::string &err = picojson::parse(result, configFile);

        if (!err.empty())
        {
            std::cout << "Can't parse JSON config: " << err << std::endl;
            return;
        }

        Parse(result);
    }

    bool ConfigManager::TryToFindConfig(const std::string &configFilePath, std::ifstream &result) const {
        const std::vector<std::string> possiblePaths = {
            configFilePath,
            std::string("../") + configFilePath,
            std::string("../../") + configFilePath,
            std::string("../../../") + configFilePath,
        };

        for (const auto &p : possiblePaths) {
            std::ifstream f(p);

            if (f.is_open())
            {
                result = std::move(f);
                return true;
            }
        }
        
        return false;
    }

    void ConfigManager::Parse(const picojson::value &v) {
        // top-level is an object
        if (v.is<picojson::object>()) {
            try {
                auto obj = v.get<picojson::object>();
                mWindowWidth = (int) obj["windowWidth"].get<double>();
                mWindowHeight = (int) obj["windowHeight"].get<double>();
                mFontScale = (float)obj["fontScale"].get<double>();
                mGuiScale = (float)obj["guiScale"].get<double>();
                mResourcesPath = obj["resourcesPath"].get<std::string>();
                mPluginPathPrefix = obj["pluginPathPrefix"].get<std::string>();

                const auto &arr = obj["pluginPaths"].get<picojson::array>();
                for (const auto &p : arr) {
                    if (p.is<std::string>()) {
                        mPluginsPaths.push_back(p.get<std::string>());
                    }
                }
            // std::map throws out_of_range, if value wasn't found
            } catch(std::out_of_range &e) {
                std::cout << "Error parsing JSON config." << std::endl;
                return;
            }
        }
    }

    glm::vec2 ConfigManager::GetWindowSize() const {
        return {mWindowWidth, mWindowHeight};
    }

    float ConfigManager::GetFontScale() const {
        return mFontScale;
    }

    float ConfigManager::GetGuiScale() const {
        return mGuiScale;
    }

    const std::string& ConfigManager::GetResourcesPath() const {
        return mResourcesPath;
    }

    const std::string & ConfigManager::GetPluginPathPrefix() const {
        return mPluginPathPrefix;
    }

    const std::vector<std::string>& ConfigManager::GetPluginsPaths() const {
        return mPluginsPaths;
    }
}
