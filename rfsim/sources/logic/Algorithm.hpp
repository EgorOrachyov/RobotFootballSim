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

#ifndef RFSIM_ALGORITHM_HPP
#define RFSIM_ALGORITHM_HPP

#include <logic/Game.hpp>
#include <rfsim/rfsim.h>
#include <dynalo/dynalo.hpp>

namespace rfsim {

    /**
     * Wrapper for dynamically loaded algorithm
     */
    class Algorithm {
    public:
        ~Algorithm();

        bool Init(dynalo::library& library);
        void GetAboutInfo(std::string& info);

        void BeginGame(Game& game);
        void TickGame(Game& game, float d);
        void EndGame(Game& game);

    private:
        rfsim_init_type initFunction{};
        rfsim_finalize_type finalizeFunction{};
        rfsim_begin_game_type beginGameFunction{};
        rfsim_tick_game_type tickGameFunction{};
        rfsim_end_game_type endGameFunction{};
        rfsim_algo_state algoState{};

        bool initialized = false;
    };

}

#endif //RFSIM_ALGORITHM_HPP