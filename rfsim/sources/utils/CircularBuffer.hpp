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

#ifndef RFSIM_CIRCULARBUFFER_HPP
#define RFSIM_CIRCULARBUFFER_HPP

#include <memory>
#include <vector>

namespace rfsim {

    template<typename T>
    class circular_buffer {
    public:

        explicit circular_buffer(size_t capacity = 0, const T& initial = T()) {
            mData.resize(capacity, initial);
        }

        circular_buffer(const circular_buffer& other) = default;
        circular_buffer(circular_buffer&& other) noexcept = default;
        ~circular_buffer() = default;

        circular_buffer& operator=(const circular_buffer& other) = default;
        circular_buffer& operator=(circular_buffer&& other) noexcept = default;

        void push_back(const T& t) {
            if (full())
                pop_front();

            mData[mTail] = t;
            mTail = (mTail + 1) % mData.size();
            mSize += 1;
        }

        void push_back(T&& t) {
            if (full())
                pop_front();

            mData[mTail] = std::move(t);
            mTail = (mTail + 1) % mData.size();
            mSize += 1;
        }

        bool pop_front(T& t) {
            if (!empty()) {
                t = std::move(mData[mHead]);
                mHead = (mHead + 1) % mData.size();
                mSize -= 1;
                return true;
            }

            return false;
        }

        bool pop_front() {
            if (!empty()) {
                mHead = (mHead + 1) % mData.size();
                mSize -= 1;
                return true;
            }

            return false;
        }

        template<typename Func>
        void for_each(Func&& func) {
            if (mSize > 0) {
                if (mHead < mTail) {
                    for (size_t i = mHead; i < mTail; i++) {
                        func(mData[i]);
                    }
                }
                else {
                    for (size_t i = mHead; i < mData.size(); i++) {
                        func(mData[i]);
                    }

                    for (size_t i = 0; i < mTail; i++) {
                        func(mData[i]);
                    }
                }
            }
        }

        template<typename Func>
        void for_each(Func&& func) const {
            if (mSize > 0) {
                if (mHead < mTail) {
                    for (size_t i = mHead; i < mTail; i++) {
                        func(mData[i]);
                    }
                }
                else {
                    for (size_t i = mHead; i < mData.size(); i++) {
                        func(mData[i]);
                    }

                    for (size_t i = 0; i < mTail; i++) {
                        func(mData[i]);
                    }
                }
            }
        }

        void resize(size_t new_size, const T& t = T()) {
            mData.resize(new_size, t);
            mHead = mTail = mSize = 0;
        }

        void clear() {
            mHead = mTail = mSize = 0;
        }

        bool full() const {
            return size() == mData.size();
        }

        bool empty() const {
            return mSize == 0;
        }

        size_t size() const {
            return mSize;
        }

    private:
        size_t mHead = 0;
        size_t mTail = 0;
        size_t mSize = 0;
        std::vector<T> mData;
    };

}

#endif //RFSIM_CIRCULARBUFFER_HPP