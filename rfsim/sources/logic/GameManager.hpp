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

#ifndef RFSIM_GAMEMANAGER_HPP
#define RFSIM_GAMEMANAGER_HPP

#include <logic/Game.hpp>
#include <logic/GameScenario.hpp>
#include <vector>

namespace rfsim {

    /**
     * @brief Game sessions manager
     *
     * Manages game scenarios and creates game sessions with selected scenarios.
     */
    class GameManager {
    public:
        GameManager() = default;
        GameManager(const GameManager& other) = delete;
        GameManager(GameManager&& other) noexcept = delete;
        ~GameManager() = default;

        void AddScenario(const std::shared_ptr<GameScenario> &scenario);
        void GetScenarioInfo(std::vector<std::string> &info);
        std::shared_ptr<Game> CreateGame(unsigned int scenarioId);

    private:
        std::vector<std::shared_ptr<GameScenario>> mScenario;
    };

}



#endif //RFSIM_GAMEMANAGER_HPP
