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

#ifndef RFSIM_GLTEXTURE_HPP
#define RFSIM_GLTEXTURE_HPP

#include <graphics/Image.hpp>
#include <GL/glew.h>

namespace rfsim {

    class GLTexture {
    public:
        GLTexture(const std::shared_ptr<Image> &image);
        ~GLTexture();

        unsigned int GetWidth() const;
        unsigned int GetHeight() const;
        unsigned int GetChannelsCount() const;
        unsigned int GetPixelSize() const;
        bool IsSRGB() const;
        const glm::uvec2 &GetSize() const;
        const std::string &GetName() const;
        GLuint GetTextureHnd() const;

    private:
        GLuint mTexture = 0;
        std::string mName;
        glm::uvec2 mSize;
        bool mIsSRGB;
        unsigned int mChannelsCount;
        unsigned int mPixelSize;
    };

}

#endif //RFSIM_GLTEXTURE_HPP