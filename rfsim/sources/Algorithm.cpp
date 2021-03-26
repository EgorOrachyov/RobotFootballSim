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

#include <Algorithm.hpp>
#include <iostream>

namespace rfsim {

    Algorithm::~Algorithm() {
        if (initialized) {
            auto finalize = finalizeFunction;
            auto status = finalize(&algoState);

            if (status != rfsim_status_success) {
                std::cerr << "Failed to finalize algorithm context "
                          << algoState.name << std::endl;
            }
        }
    }
    
    bool Algorithm::Init(dynalo::library &library) {
        initFunction = library.get_function<rfsim_status(rfsim_algo_state*)>(RFSIM_FUNCTION_INIT_NAME);
        beginGameFunction = library.get_function<rfsim_status(rfsim_algo_state*, const rfsim_game_settings*, const rfsim_game_start_info*)>(RFSIM_FUNCTION_BEGIN_GAME_NAME);
        tickGameFunction = library.get_function<rfsim_status(rfsim_algo_state*, rfsim_game_state_info*)>(RFSIM_FUNCTION_TICK_GAME_NAME);
        endGameFunction = library.get_function<rfsim_status(rfsim_algo_state*)>(RFSIM_FUNCTION_END_GAME_NAME);
        finalizeFunction = library.get_function<rfsim_status(rfsim_algo_state*)>(RFSIM_FUNCTION_FINALIZE_NAME);

        auto init = initFunction;
        auto status = init(&algoState);

        return initialized = status == rfsim_status_success;
    }

    void Algorithm::GetAboutInfo(std::string &info) {
        info = algoState.name;
    }

    void Algorithm::BeginGame() {

    }

    void Algorithm::TickGame() {

    }

    void Algorithm::EndGame() {

    }

}