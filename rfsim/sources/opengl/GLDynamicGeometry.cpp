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

#include <opengl/GLDynamicGeometry.hpp>

namespace rfsim {

    GLDynamicGeometry::GLDynamicGeometry(size_t vertexBuffersCount, bool useIndices, GLenum topology,
                                         GLGeometryLayout &&layout)
         : mTopology(topology),
           mVertexBuffersCount(vertexBuffersCount),
           mLayout(std::move(layout)) {

        assert(vertexBuffersCount <= MAX_VERTEX_BUFFERS);

        for (size_t i = 0; i < mVertexBuffersCount; i++) {
            mVertexBuffers[i] = std::make_shared<GLDynamicVertexBuffer>();
        }

        if (useIndices)
            mIndexBuffer = std::make_shared<GLDynamicIndexBuffer>();
    }

    GLDynamicGeometry::~GLDynamicGeometry() {
        ReleaseVAO();
    }

    const std::shared_ptr<GLDynamicVertexBuffer> &GLDynamicGeometry::GetBuffer(unsigned int index) const {
        assert(index < mVertexBuffersCount);
        return mVertexBuffers[index];
    }

    const std::array<std::shared_ptr<GLDynamicVertexBuffer>, GLDynamicGeometry::MAX_VERTEX_BUFFERS> & GLDynamicGeometry::GetBuffers() const {
        return mVertexBuffers;
    }

    const std::shared_ptr<GLDynamicIndexBuffer> & GLDynamicGeometry::GetIndexBuffer() const {
        return mIndexBuffer;
    }

    void GLDynamicGeometry::UpdateResource() {
        for (size_t i = 0; i < mVertexBuffersCount; i++) {
            mVertexBuffers[i]->UpdateResource();
        }

        if (mIndexBuffer != nullptr)
            mIndexBuffer->UpdateResource();

        RecreateVAO();
    }

    void GLDynamicGeometry::ClearGeometry() {
        for (size_t i = 0; i < mVertexBuffersCount; i++) {
            mVertexBuffers[i]->Clear();
        }

        if (mIndexBuffer != nullptr)
            mIndexBuffer->Clear();
    }

    void GLDynamicGeometry::Bind() {
        assert(mVAO);
        glBindVertexArray(mVAO);
    }

    void GLDynamicGeometry::Unbind() {
        glBindVertexArray(0);
    }

    void GLDynamicGeometry::Draw(size_t indicesCount, size_t instancesCount) const {
        glDrawElementsInstanced(mTopology, indicesCount, GL_UNSIGNED_INT, nullptr, instancesCount);
    }

    GLuint GLDynamicGeometry::GetHnd() const {
        return mVAO;
    }

    GLenum GLDynamicGeometry::GetTopology() const {
        return mTopology;
    }

    void GLDynamicGeometry::ReleaseVAO() {
        if (mVAO) {
            glDeleteVertexArrays(1, &mVAO);
            mVAO = 0;
        }
    }

    void GLDynamicGeometry::RecreateVAO() {
        ReleaseVAO();

        glGenVertexArrays(1, &mVAO);
        glBindVertexArray(mVAO);

        for (size_t bufferId = 0; bufferId < mVertexBuffersCount; bufferId++) {
            glBindBuffer(GL_ARRAY_BUFFER, mVertexBuffers[bufferId]->GetBufferHnd());

            for (const auto& element: mLayout) {
                if (element.bufferId == bufferId) {
                    glEnableVertexAttribArray(element.location);
                    glVertexAttribDivisor(element.location, element.perInstance ? 1 : 0);
                    glVertexAttribPointer(element.location, element.components, element.baseType, element.normalize ? GL_TRUE : GL_FALSE, element.stride, (void*) element.offset);
                }
            }
        }

        if (mIndexBuffer != nullptr)
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mIndexBuffer->GetBufferHnd());

        glBindVertexArray(GL_NONE);
        glBindBuffer(GL_ARRAY_BUFFER, GL_NONE);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, GL_NONE);
    }

}