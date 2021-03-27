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

#ifndef RFSIM_ALGORITHMMANAGER_HPP
#define RFSIM_ALGORITHMMANAGER_HPP

#include <logic/Algorithm.hpp>
#include <dynalo/dynalo.hpp>
#include <vector>
#include <memory>
#include <string>

namespace rfsim {

    class AlgorithmManager {
    public:
        explicit AlgorithmManager(const std::string& prefixPath);
        AlgorithmManager(const AlgorithmManager& other) = delete;
        AlgorithmManager(AlgorithmManager&& other) noexcept = delete;
        ~AlgorithmManager();

        std::shared_ptr<Algorithm> Load(const std::string& name);
        std::shared_ptr<Algorithm> LoadFromFilepath(const std::string& filepath);
        std::shared_ptr<Algorithm> GetAlgorithmAt(unsigned int i);
        void GetAlgorithmsInfo(std::vector<std::string> &info);

    private:
        std::string mPrefixPath;
        std::vector<std::shared_ptr<Algorithm>> mAlgorithms;
        std::vector<std::shared_ptr<dynalo::library>> mLibs;
    };

}

#endif //RFSIM_ALGORITHMMANAGER_HPP