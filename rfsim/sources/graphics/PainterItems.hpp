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
#include <graphics/Painter.hpp>

namespace rfsim {

    struct PainterItem {
        Painter::Color penColor;
        Painter::Color brushColor;
        Painter::Color transparentColor;
        bool filled;
        unsigned int penWidth;
        int zOrder;

        PainterItem(const Painter::Palette& palette, int z) {
            penColor = palette.penColor;
            brushColor = palette.brushColor;
            transparentColor = palette.transparentColor;
            filled = palette.filled;
            penWidth = palette.penWidth;
            zOrder = z;
        }
    };

    struct PainterRect: public PainterItem {
        Painter::Rect rect;
        float angle;

        PainterRect(const Painter::Rect& r, float a,
                    const Painter::Palette& palette, int z)
            : PainterItem(palette, z) {
            rect = r;
            angle = a;
        }

        void PrepareRect(GLDynamicGeometry& geometry, size_t baseIndex, size_t &indicesToDraw, size_t &verticesToDraw) const {
            const size_t FRAME_VERTICES_COUNT = 4;
            const size_t VERTICES_COUNT_FILLED = 4;
            const size_t VERTICES_COUNT_NOT_FILLED = 8;
            const size_t INDICES_COUNT_FILLED = 3 * 2;
            const size_t INDICES_COUNT_NOT_FILLED = 3 * 8;

            glm::vec3 framePositions[FRAME_VERTICES_COUNT];
            FillAndRotateRect(framePositions, FRAME_VERTICES_COUNT);

            auto& colorData = geometry.GetBuffer(1);
            auto& vertexData = geometry.GetBuffer(0);
            auto& indexData = geometry.GetIndexBuffer();

            // If borders have actually more width, than rect itself, then draw filled rect
            auto checkFilled = filled || ((2.0f * (float) penWidth) >= rect.z) || ((2.0f * (float) penWidth) >= rect.w);

            // Set 1.0f in vertex attribute
            auto fFilled = checkFilled? 1.0f: 0.0f;

            // Base offset, since rendered in batch
            auto bi = (uint32_t) baseIndex;

            verticesToDraw = checkFilled? VERTICES_COUNT_FILLED: VERTICES_COUNT_NOT_FILLED;

            for (size_t i = 0; i < verticesToDraw; i++) {
                colorData->Append((const unsigned char*)&penColor, sizeof(penColor));
                colorData->Append((const unsigned char*)&brushColor, sizeof(brushColor));
                colorData->Append((const unsigned char*)&fFilled, sizeof(float));
            }

            if (checkFilled) {
                //
                // v0------------v3
                // |              |
                // v1------------v2
                //

                uint32_t indices[INDICES_COUNT_FILLED] = {
                    bi + 0, bi + 1, bi + 2,
                    bi + 2, bi + 3, bi + 0
                };

                vertexData->Append((const uint8_t*)framePositions, sizeof(framePositions));
                indexData->Append(indices, INDICES_COUNT_FILLED);
                indicesToDraw = INDICES_COUNT_FILLED;
            }
            else {
                auto w = (float) penWidth;

                //
                // v0------------v3
                // | v4--------v7 |
                // | |          | |
                // | v5--------v6 |
                // v1------------v2
                //

                glm::vec3 innerFramePositions[FRAME_VERTICES_COUNT];
                FillRect(innerFramePositions, FRAME_VERTICES_COUNT);

                innerFramePositions[0] += glm::vec3(w, w, 0);
                innerFramePositions[1] += glm::vec3(w, -w, 0);
                innerFramePositions[2] += glm::vec3(-w, -w, 0);
                innerFramePositions[3] += glm::vec3(-w, w, 0);

                RotateRect(innerFramePositions, FRAME_VERTICES_COUNT);

                glm::vec3 positions[VERTICES_COUNT_NOT_FILLED] = {
                    framePositions[0],
                    framePositions[1],
                    framePositions[2],
                    framePositions[3],
                    innerFramePositions[0],
                    innerFramePositions[1],
                    innerFramePositions[2],
                    innerFramePositions[3],
                };

                uint32_t indices[INDICES_COUNT_NOT_FILLED] = {
                    bi + 0, bi + 1, bi + 5,
                    bi + 0, bi + 5, bi + 4,
                    bi + 1, bi + 2, bi + 6,
                    bi + 1, bi + 6, bi + 5,
                    bi + 2, bi + 3, bi + 7,
                    bi + 2, bi + 7, bi + 6,
                    bi + 3, bi + 0, bi + 4,
                    bi + 3, bi + 4, bi + 7
                };

                vertexData->Append((const uint8_t*)positions, sizeof(positions));
                indexData->Append(indices, INDICES_COUNT_NOT_FILLED);
                indicesToDraw = INDICES_COUNT_NOT_FILLED;
            }
        }

        void RotateRect(glm::vec3* positions, size_t count) const {
            assert(count == 4);

            glm::vec2 corner = glm::vec2{rect.x, rect.y};
            glm::vec2 size = glm::vec2{rect.z, rect.w};
            glm::vec3 center = glm::vec3(corner + size * 0.5f, 0.0);

            // angle defines clockwise rotation
            glm::mat4 rotation = glm::rotate(angle, glm::vec3(0.0f, 0.0f, 1.0f));

            for (size_t i = 0; i < count; i++) {
                glm::vec3& p = positions[i];

                glm::vec4 relative = glm::vec4((p - center), 0.0f);
                p = center + glm::vec3(rotation * relative);
                p.z = (float) zOrder;
            }
        }

        void FillRect(glm::vec3* positions, size_t count) const {
            assert(count == 4);

            glm::vec2 size = glm::vec2{rect.z, rect.w};

            positions[0] = glm::vec3{rect.x, rect.y, 0};
            positions[1] = glm::vec3{rect.x, rect.y + size.y, 0};
            positions[2] = glm::vec3{rect.x + size.x, rect.y + size.y, 0};
            positions[3] = glm::vec3{rect.x + size.x, rect.y, 0};
        }

        void FillAndRotateRect(glm::vec3* positions, size_t count) const {
            FillRect(positions, count);
            RotateRect(positions, count);
        }
    };

    struct PainterImage: public PainterRect {
        std::shared_ptr<GLTexture> image;

        PainterImage(const std::shared_ptr<GLTexture> &i,
                     const Painter::Rect& r, float a,
                     const Painter::Palette& palette, int z)
                : PainterRect(r, a, palette, z) {
            image = i;
        }

        void PrepareImage(GLDynamicGeometry& geometry, size_t baseIndex, size_t &indicesToDraw, size_t& verticesToDraw) const {
            const size_t VERTICES_COUNT = 4;
            const size_t INDICES_COUNT = 6;

            indicesToDraw = INDICES_COUNT;
            verticesToDraw = VERTICES_COUNT;

            glm::vec3 positions[VERTICES_COUNT];
            FillAndRotateRect(positions, VERTICES_COUNT);

            glm::vec2 texCoords[VERTICES_COUNT] = {
                {0.0f, 1.0f},
                {0.0f, 0.0f},
                {1.0f, 0.0f},
                {1.0f, 1.0f},
            };

            uint32_t bi = baseIndex;

            uint32_t indices[INDICES_COUNT] = {
                bi + 0, bi + 1, bi + 2,
                bi + 2, bi + 3, bi + 0
            };

            auto& vertexData = geometry.GetBuffer(0);

            for (size_t i = 0; i < VERTICES_COUNT; i++) {
                vertexData->Append((const unsigned char*)(&positions[i]), sizeof(positions[i]));
                vertexData->Append((const unsigned char*)(&texCoords[i]), sizeof(texCoords[i]));
            }

            float srgb = image->IsSRGB()? 1.0f: 0.0f;

            auto& sharedData = geometry.GetBuffer(1);

            for (size_t i = 0; i < verticesToDraw; i++) {
                sharedData->Append((const unsigned char*)&brushColor, sizeof(brushColor));
                sharedData->Append((const unsigned char*)&transparentColor, sizeof(transparentColor));
                sharedData->Append((const unsigned char*)&srgb, sizeof(srgb));
            }

            auto& indexData = geometry.GetIndexBuffer();
            indexData->Append(indices, INDICES_COUNT);
        }
    };

}

#endif //RFSIM_PAINTERITEMS_HPP
