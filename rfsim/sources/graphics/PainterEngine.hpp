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

#ifndef RFSIM_PAINTERENGINE_HPP
#define RFSIM_PAINTERENGINE_HPP

#include <graphics/Image.hpp>
#include <graphics/Window.hpp>
#include <glm/vec2.hpp>
#include <glm/vec4.hpp>
#include <memory>

namespace rfsim {

    /**
     * Painter for drawing 2d primitives.
     *
     * Use coordinate space, defined as follows.
     *
     *                              x-axis
     *        (originX,originY)------------>originX+w
     *        |
     *        |
     *        |
     * y-axis |
     *        |
     *     originY+h
     */
    class PainterEngine {
    public:
        using Size = glm::vec2;
        using Point = glm::vec2;
        using Rect = glm::vec4;
        using Recti = glm::ivec4;
        using Color = glm::vec4;

        struct Palette {
            Color penColor{0.0f, 0.0f, 0.0f, 0.0f};
            Color brushColor{0.0f, 0.0f, 0.0f, 0.0f};
            Color transparentColor{2.0f, 2.0f, 2.0f, 2.0f};
            bool filled = false;
            unsigned int penWidth = 1;
        };

        /**
         * Creates painter engine.
         *
         * @param area Area in pixels of the target screen, used for drawing.
         * @param space Virtual coordinates space mapped to the screen.
         * @param target Target window for drawing.
         */
        PainterEngine(const Recti& area, const Rect& space, std::shared_ptr<Window> target);
        PainterEngine(const PainterEngine& engine) = delete;
        PainterEngine(PainterEngine&& engine) noexcept = delete;
        ~PainterEngine() = default;

        void DrawLine(const Point& from, const Point& to);

        /**
         * Draw rect on the paint device.
         *
         * @note If set filling=true rect is filled with brush color
         * @note If set filling=false rect is drawn as frame with pen color and pen width sides
         *
         * @param rect Rect shape (top left corner, width and height)
         * @param angle Clockwise rotation of the rect (relative to rect center)
         */
        void DrawRect(const Rect& rect, float angle);

        void DrawEllipse(const Point& center, float radiusX, float radiusY);
        void DrawCircle(const Point& center, float radius);

        /**
         * Draw image on top of the paint device.
         *
         * @note Image color is multiplied to brush color
         * @note Transparent color used to discard pixels
         *
         * @param target Rect, which defines bounds of the drawn image
         * @param angle Clockwise rotation in radians of the target rect
         * @param image Image to draw. Must be in RGBA 8-bit per component format.
         */
        void DrawImage(const Rect& target, float angle, const std::shared_ptr<Image> &image);

        /**
         * Clear Paint area. All drawn objects will be removed.
         */
        void Clear();

        /**
         * Issue actual drawing on OpenGL side.
         */
        void Draw();

        /**
         * Resize Draw Area to the framebuffer size
         */
        void FitToFramebufferArea();

        void SetDrawArea(const Recti& area);
        void SetDrawSpace(const Rect& space);
        void SetPenColor(const Color& color);
        void SetPenWidth(unsigned int width);
        void SetBrushColor(const Color& color);
        void SetTransparentColor(const Color& color);
        void SetFilling(bool fill);
        void SetClearColor(const Color& color);
        void SetDrawPalette(const Palette& palette);

        const Recti& GetDrawArea() const;
        const Rect& GetDrawSpace() const;
        const Palette& GetDrawPalette() const;
        const std::shared_ptr<Window> &GetWindow() const;

    private:
        Recti mArea;
        Rect mSpace;
        Palette mPalette;
        Color mClearColor;
        std::shared_ptr<Window> mWindow;
        std::shared_ptr<class PainterEnginePrivate> mPrivate;
    };

}

#endif //RFSIM_PAINTERENGINE_HPP