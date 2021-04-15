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

#include <opengl/GLTexture.hpp>

namespace rfsim {

    GLTexture::GLTexture(const std::shared_ptr<Image> &image) {
        assert(image != nullptr);

        mName = image->GetName();
        mSize = image->GetSize();
        mChannelsCount = image->GetChannelsCount();
        mPixelSize = image->GetPixelSize();
        mIsSRGB = image->IsSRGB();

        assert(mPixelSize == 4);
        assert(mChannelsCount == 4);

        glGenTextures(1, &mTexture);
        glBindTexture(GL_TEXTURE_2D, mTexture);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, mSize.x, mSize.y, 0, GL_RGBA, GL_UNSIGNED_BYTE, image->GetPixelData().data());
        glGenerateMipmap(GL_TEXTURE_2D);

        // Set best filtering options for each texture
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        glBindTexture(GL_TEXTURE_2D, 0);
    }

    GLTexture::~GLTexture() {
        if (mTexture) {
            glDeleteTextures(1, &mTexture);
            mTexture = 0;
        }
    }

    unsigned int GLTexture::GetWidth() const {
        return mSize.x;
    }

    unsigned int GLTexture::GetHeight() const {
        return mSize.y;
    }

    unsigned int GLTexture::GetChannelsCount() const {
        return mChannelsCount;
    }

    unsigned int GLTexture::GetPixelSize() const {
        return mPixelSize;
    }

    bool GLTexture::IsSRGB() const {
        return mIsSRGB;
    }

    const glm::uvec2 & GLTexture::GetSize() const {
        return mSize;
    }

    const std::string & GLTexture::GetName() const {
        return mName;
    }

    GLuint GLTexture::GetTextureHnd() const {
        return mTexture;
    }

}