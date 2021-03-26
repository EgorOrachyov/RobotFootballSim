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

#include <AlgorithmManager.hpp>
#include <iostream>
#include <cassert>

namespace rfsim {

    AlgorithmManager::AlgorithmManager(const std::string &prefixPath) {
        mPrefixPath = prefixPath;
    }

    AlgorithmManager::~AlgorithmManager() {
        mAlgorithms.clear();
        mLibs.clear();
    }

    std::shared_ptr<Algorithm> AlgorithmManager::Load(const std::string& name) {
        const auto SEP = "/";
        std::string libPath = mPrefixPath + SEP + dynalo::to_native_name(name);
        std::shared_ptr<dynalo::library> lib = std::make_shared<dynalo::library>(libPath);

        std::shared_ptr<Algorithm> algo = std::make_shared<Algorithm>();

        if (algo->Init(*lib)) {
            std::cout << "AlgorithmManager: load plugin: " << libPath << std::endl;
            mAlgorithms.push_back(algo);
            mLibs.push_back(lib);
            return algo;
        }

        return nullptr;
    }

    std::shared_ptr<Algorithm> AlgorithmManager::GetAlgorithmAt(unsigned int i) {
        assert(i < mAlgorithms.size());
        return mAlgorithms[i];
    }

    void AlgorithmManager::GetAlgorithmsInfo(std::vector<std::string> &info) {
        info.clear();
        info.reserve(mAlgorithms.size());

        for (auto& algo: mAlgorithms) {
            std::string text;
            algo->GetAboutInfo(text);
            info.push_back(std::move(text));
        }
    }

}
