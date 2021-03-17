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

#ifndef RFSIM_IMAGE_HPP
#define RFSIM_IMAGE_HPP

#include <glm/vec2.hpp>
#include <vector>
#include <string>
#include <memory>
#include <cinttypes>

namespace rfsim {

    /**
     * RGBA Image class.
     */
    class Image {
    public:
        Image(std::string&& name, const glm::uvec2& size, std::vector<uint8_t> &&data, unsigned int channelsCount, unsigned int pixelSize, bool isSRGB);
        Image(const Image& image) = default;
        Image(Image&& image) noexcept = default;
        ~Image() = default;

        unsigned int GetWidth() const;
        unsigned int GetHeight() const;
        unsigned int GetChannelsCount() const;
        unsigned int GetPixelSize() const;
        bool IsSRGB() const;
        const glm::uvec2 &GetSize() const;
        const std::string &GetName() const;
        const std::vector<uint8_t> &GetPixelData() const;

        /**
         * @brief Load image from file.
         * Loads image in RGBA format, with 4 bytes per pixel and 8 bit depth per color component.
         * Supported formats: png, jpeg, svg, bmp, tga, etc. Refer to stb_image.
         *
         * @param filePath Qualified path to file with image.
         * @param isSRGB Pass true if image in SRGB format
         * @return Image or null pointer if failed to load.
         */
        static std::shared_ptr<Image> LoadFromFilePath(std::string &&filePath, bool isSRGB = true);

    private:
        std::string mName;
        std::vector<uint8_t> mData;
        glm::uvec2 mSize;
        bool mIsSRGB;
        unsigned int mChannelsCount;
        unsigned int mPixelSize;
    };

}

#endif //RFSIM_IMAGE_HPP