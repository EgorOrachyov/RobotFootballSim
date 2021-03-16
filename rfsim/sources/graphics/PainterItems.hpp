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

#ifndef RFSIM_PAINTERITEMS_HPP
#define RFSIM_PAINTERITEMS_HPP

#include <glm/glm.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <opengl/GLTexture.hpp>
#include <opengl/GLDynamicGeometry.hpp>
#include <graphics/PainterEngine.hpp>

namespace rfsim {

    struct PainterItem {
        PainterEngine::Color penColor;
        PainterEngine::Color brushColor;
        bool filled;
        unsigned int penWidth;
        int zOrder;

        PainterItem(const PainterEngine::Palette& palette, int z) {
            penColor = palette.penColor;
            brushColor = palette.brushColor;
            filled = palette.filled;
            penWidth = palette.penWidth;
            zOrder = z;
        }
    };

    struct PainterRect: public PainterItem {
        PainterEngine::Rect rect;
        float angle;

        PainterRect(const PainterEngine::Rect& r, float a,
                    const PainterEngine::Palette& palette, int z)
            : PainterItem(palette, z) {
            rect = r;
            angle = a;
        }


    };

    struct PainterImage: public PainterRect {
        std::shared_ptr<GLTexture> image;

        PainterImage(const std::shared_ptr<GLTexture> &i,
                     const PainterEngine::Rect& r, float a,
                     const PainterEngine::Palette& palette, int z)
                : PainterRect(r, a, palette, z) {
            image = i;
        }

        void PrepareImage(GLDynamicGeometry& geometry) const {
            const size_t VERTICES_COUNT = 4;
            const size_t INDICES_COUNT = 6;

            glm::vec2 corner = glm::vec2{rect.x, rect.y};
            glm::vec2 size = glm::vec2{rect.z, rect.w};
            glm::vec3 center = glm::vec3(corner + size * 0.5f, 0.0);

            auto rotation = glm::rotate(angle, glm::vec3{0, 0, 1.0});

            glm::vec3 positions[VERTICES_COUNT] = {
                {rect.x, rect.y, 0},
                {rect.x, rect.y + size.y, 0},
                {rect.x + size.x, rect.y + size.y, 0},
                {rect.x + size.x, rect.y, 0},
            };

            glm::vec2 texCoords[VERTICES_COUNT] = {
                {0.0f, 1.0f},
                {0.0f, 0.0f},
                {1.0f, 0.0f},
                {1.0f, 1.0f},
            };

            uint32_t indices[INDICES_COUNT] = {
                0, 1, 2,
                2, 3, 0
            };

            for (auto& p: positions) {
                glm::vec4 relative = glm::vec4((p - center), 0.0f);
                p = center + glm::vec3(rotation * relative);
                p.z = (float) zOrder;
            }

            auto& vertexData = geometry.GetBuffer(0);

            for (size_t i = 0; i < VERTICES_COUNT; i++) {
                vertexData->Append((const unsigned char*)(&positions[i]), sizeof(positions[i]));
                vertexData->Append((const unsigned char*)(&texCoords[i]), sizeof(texCoords[i]));
            }

            auto& colorData = geometry.GetBuffer(1);
            colorData->Append((const unsigned char*)&brushColor, sizeof(brushColor));

            auto& indexData = geometry.GetIndexBuffer();
            indexData->Append(indices, INDICES_COUNT);
        }
    };

}

#endif //RFSIM_PAINTERITEMS_HPP
