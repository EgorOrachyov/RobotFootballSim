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

#ifndef RFSIM_CONFIG_MANAGER_HPP
#define RFSIM_CONFIG_MANAGER_HPP

#include <string>
#include <vector>

namespace picojson {
    class value;
}

namespace rfsim
{

    /**
     * @brief Configuration files manager.
     *
     * Manages configuration parsing and getting values.
     */
    class ConfigManager {
    public:
        ConfigManager(const std::string &configFilePath);
        ~ConfigManager() = default;

        ConfigManager(const ConfigManager &other) = delete;
        ConfigManager(ConfigManager &&other) noexcept = delete;
        ConfigManager& operator=(const ConfigManager &other) = delete;
        ConfigManager& operator=(ConfigManager &&other) noexcept = delete;

        float GetFontScale() const;
        float GetGuiScale() const;
        const std::string& GetResourcesPath() const;
        const std::string& GetPluginPathPrefix() const;
        const std::vector<std::string>& GetPluginsPaths() const;

    private:
        bool TryToFindConfig(const std::string &configFilePath, std::ifstream &result) const;
        void Parse(const picojson::value &v);

    private:
        float mFontScale;
        float mGuiScale;
        std::string mResourcesPath;
        std::string mPluginPathPrefix;
        std::vector<std::string> mPluginsPaths;
    };

}

#endif // RFSIM_CONFIG_MANAGER_HPP
