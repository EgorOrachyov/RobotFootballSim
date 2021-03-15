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
     */
    class PainterEngine {
    public:
        using Size = glm::vec2;
        using Point = glm::vec2;
        using Rect = glm::vec4;
        using Color = glm::vec4;

        struct Palette {
            Color PenColor{0.0f, 0.0f, 0.0f, 0.0f};
            Color BrushColor{0.0f, 0.0f, 0.0f, 0.0f};
            bool Filled = false;
        };

        PainterEngine(const Rect& area, std::shared_ptr<Window> target);
        PainterEngine(const PainterEngine& engine) = delete;
        PainterEngine(PainterEngine&& engine) noexcept = delete;
        ~PainterEngine() = default;

        void DrawLine(const Point& from, const Point& to);
        void DrawRect(const Rect& rect);
        void DrawEllipse(const Point& center, float radiusX, float radiusY);
        void DrawCircle(const Point& center, float radius);
        void DrawImage(const Rect& target, float angle, const std::shared_ptr<Image> &image);
        void Clear();

        void Draw();

        void SetDrawArea(const Rect& area);
        void SetPenColor(const Color& color);
        void SetBrushColor(const Color& color);
        void SetFilling(bool fill);
        void SetDrawPalette(const Palette& palette);

        const Rect& GetDrawArea() const;
        const Palette& GetDrawPalette() const;
        const std::shared_ptr<Window> &GetWindow() const;

    private:
        Rect mArea;
        Palette mPalette;
        std::shared_ptr<Window> mWindow;
        std::shared_ptr<class PainterEnginePrivate> mPrivate;
    };

}

#endif //RFSIM_PAINTERENGINE_HPP