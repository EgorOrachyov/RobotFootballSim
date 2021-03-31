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

#ifndef RFSIM_GAMERULESMANAGER_HPP
#define RFSIM_GAMERULESMANAGER_HPP

#include <logic/GameRule.hpp>
#include <vector>
#include <memory>

namespace rfsim {

    class GameRulesManager {
    public:
        GameRulesManager() = default;
        GameRulesManager(const GameRulesManager& other) = delete;
        GameRulesManager(GameRulesManager&& other) noexcept = delete;
        ~GameRulesManager() = default;

        void GetRulesInfo(std::vector<std::string> &info);
        void AddRule(const std::shared_ptr<GameRule> &rule);
        std::shared_ptr<GameRule> GetRule(unsigned int idx);

    private:
        std::vector<std::shared_ptr<GameRule>> mRules;
    };

}

#endif //RFSIM_GAMERULESMANAGER_HPP