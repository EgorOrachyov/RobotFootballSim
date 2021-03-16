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

#ifndef RFSIM_GLDYNAMICGEOMETRY_HPP
#define RFSIM_GLDYNAMICGEOMETRY_HPP

#include <opengl/GLDynamicBuffer.hpp>
#include <opengl/GLGeometryLayout.hpp>
#include <memory>
#include <array>

namespace rfsim {

    /**
     * Dynamic OpenGL geometry with fixed topology and vertices layout.
     * Allows dynamically update content and recreate object, if needed.
     */
    class GLDynamicGeometry {
    public:
        // For our tasks its enough. Default GL constant is 8 (or 16 on some platforms)
        static const unsigned int MAX_VERTEX_BUFFERS = 4;

        GLDynamicGeometry(size_t vertexBuffersCount, bool useIndices, GLenum topology, GLGeometryLayout&& layout);
        GLDynamicGeometry(const GLDynamicGeometry& other) = delete;
        GLDynamicGeometry(GLDynamicGeometry&& other) noexcept = delete;
        ~GLDynamicGeometry();

        const std::shared_ptr<GLDynamicVertexBuffer> &GetBuffer(unsigned int index) const;
        const std::array<std::shared_ptr<GLDynamicVertexBuffer>, MAX_VERTEX_BUFFERS> &GetBuffers() const;
        const std::shared_ptr<GLDynamicIndexBuffer> &GetIndexBuffer() const;

        void UpdateResource();
        void ClearGeometry();
        void Bind();
        void Unbind();
        void Draw(size_t indicesCount, size_t instancesCount) const;

        GLuint GetHnd() const;
        GLenum GetTopology() const;

    private:
        void ReleaseVAO();
        void RecreateVAO();

        GLuint mVAO = 0;
        GLenum mTopology;
        GLGeometryLayout mLayout;
        size_t mVertexBuffersCount;
        std::array<std::shared_ptr<GLDynamicVertexBuffer>, MAX_VERTEX_BUFFERS> mVertexBuffers;
        std::shared_ptr<GLDynamicIndexBuffer> mIndexBuffer;
    };

}




#endif //RFSIM_GLDYNAMICGEOMETRY_HPP
