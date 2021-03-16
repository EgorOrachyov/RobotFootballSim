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

#include <graphics/Image.hpp>
#include <stb/stb_image.hpp>
#include <utility>
#include <cstring>

namespace rfsim {

    Image::Image(std::string &&name, const glm::uvec2 &size, std::vector<uint8_t> &&data, unsigned int channelsCount, unsigned int pixelSize)
        : mName(std::move(name)),
          mSize(size),
          mData(std::move(data)),
          mChannelsCount(channelsCount),
          mPixelSize(pixelSize) {
        assert(pixelSize > 0);
        assert(channelsCount > 0);
        assert(size.x * size.y * pixelSize == mData.size());
    }

    unsigned int Image::GetWidth() const {
        return mSize.x;
    }

    unsigned int Image::GetHeight() const {
        return mSize.y;
    }

    unsigned int Image::GetChannelsCount() const {
        return mChannelsCount;
    }

    unsigned int Image::GetPixelSize() const {
        return mPixelSize;
    }

    const glm::uvec2 & Image::GetSize() const {
        return mSize;
    }

    const std::string & Image::GetName() const {
        return mName;
    }

    const std::vector<uint8_t> & Image::GetPixelData() const {
        return mData;
    }

    std::shared_ptr<Image> Image::LoadFromFilePath(std::string &&filePath) {
        std::shared_ptr<Image> result;

        int width = 0;
        int height = 0;
        int channels = 4;
        int desiredChannelsCount = 4;

        const unsigned char* data = stbi_load(filePath.c_str(), &width, &height, &channels, desiredChannelsCount);

        if (data != nullptr) {
            assert(width > 0);
            assert(height > 0);
            assert(channels == desiredChannelsCount);

            std::vector<uint8_t> pixelData;
            pixelData.resize(width * height * channels);
            std::memcpy(pixelData.data(), data, width * height * channels);

            result = std::make_shared<Image>(std::move(filePath), glm::uvec2(width, height), std::move(pixelData), channels, channels);
        }

        if (data) {
            stbi_image_free((void *) data);
        }

        return result;
    }

}