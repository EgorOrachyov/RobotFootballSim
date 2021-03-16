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

#ifndef RFSIM_GLDYNAMICBUFFER_HPP
#define RFSIM_GLDYNAMICBUFFER_HPP

#include <GL/glew.h>
#include <vector>
#include <type_traits>
#include <cassert>
#include <cstring>

namespace rfsim {

    /**
     * Generic dynamic OpenGL buffer.
     * Allows dynamically fill content with variable size and sync these data with actual GPU buffer.
     *
     * @tparam GLBufferType Type of the OpenGL buffer (GL_ARRAY_BUFFER or GL_ELEMENTS_ARRAY_BUFFER)
     * @tparam StoredDataType Type of stored data (must be trivially copyable)
     */
    template <GLenum GLBufferType, typename StoredDataType>
    class GLDynamicBuffer {
    public:
        static_assert(std::is_trivially_copyable<StoredDataType>::value, "Must be trivially copyable");
        static_assert(GLBufferType == GL_ARRAY_BUFFER || GLBufferType == GL_ELEMENT_ARRAY_BUFFER, "Must be GL_ARRAY_BUFFER or GL_ELEMENTS_ARRAY_BUFFER");

        static const GLenum BufferType = GLBufferType;
        using DataType = StoredDataType;

        GLDynamicBuffer() = default;
        GLDynamicBuffer(const GLDynamicBuffer& other) = delete;
        GLDynamicBuffer(GLDynamicBuffer&& other) noexcept = delete;
        ~GLDynamicBuffer() {
            ReleaseHnd();
        }

        void Append(const DataType* data, size_t size) {
            assert(data);
            assert(size > 0);

            auto oldSize = mCache.size();
            mCache.resize(oldSize + size);
            std::memcpy(mCache.data() + oldSize, data, size * sizeof(DataType));
            mDirty = true;
        }

        void Clear() {
            mCache.clear();
            mDirty = true;
        }

        void UpdateResource() {
            if (mDirty && !mCache.empty())  {
                size_t byteSize = mCache.size() * sizeof(DataType);

                if (byteSize  > mGpuBufferSize)
                    ReallocateGpuBuffer(byteSize);

                glBindBuffer(BufferType, mGpuBuffer);
                glBufferSubData(BufferType, 0, byteSize, mCache.data());
                glBindBuffer(BufferType, 0);
            }

            mDirty = false;
        }

        size_t GetSize() const {
            return mCache.size();
        }

        GLuint GetBufferHnd() const {
            return mGpuBuffer;
        }

    private:

        void ReleaseHnd() {
            if (mGpuBuffer) {
                glDeleteBuffers(1, &mGpuBuffer);
                mGpuBuffer = 0;
            }
        }

        void ReallocateGpuBuffer(size_t size) {
            if (mGpuBufferSize == 0)
                mGpuBufferSize = mGpuBufferInitialSize;

            while (mGpuBufferSize < size)
                mGpuBufferSize *= 2;

            ReleaseHnd();

            glGenBuffers(1, &mGpuBuffer);
            glBindBuffer(BufferType, mGpuBuffer);
            glBufferData(BufferType, mGpuBufferSize, nullptr, GL_DYNAMIC_DRAW);
            glBindBuffer(BufferType, 0);
        }

        std::vector<DataType> mCache;
        GLuint mGpuBuffer = 0;
        size_t mGpuBufferSize = 0;
        size_t mGpuBufferInitialSize = 64;
        bool mDirty = true;
    };

    using GLDynamicVertexBuffer = GLDynamicBuffer<GL_ARRAY_BUFFER, uint8_t>;
    using GLDynamicIndexBuffer = GLDynamicBuffer<GL_ELEMENT_ARRAY_BUFFER, uint32_t>;

}

#endif //RFSIM_GLDYNAMICBUFFER_HPP