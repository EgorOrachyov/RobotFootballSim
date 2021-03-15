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

#include <graphics/PainterEngine.hpp>
#include <utility>

namespace rfsim {

    class PainterEnginePrivate {
    public:

    };

    PainterEngine::PainterEngine(const Rect &area, std::shared_ptr<Window> target)
        : mArea(area),
          mWindow(std::move(target)) {

    }

    void PainterEngine::DrawLine(const Point &from, const Point &to) {

    }

    void PainterEngine::DrawRect(const Rect &rect) {

    }

    void PainterEngine::DrawEllipse(const Point &center, float radiusX, float radiusY) {

    }

    void PainterEngine::DrawCircle(const Point &center, float radius) {

    }

    void PainterEngine::DrawImage(const Rect &target, float angle, const std::shared_ptr<Image> &image) {

    }

    void PainterEngine::Clear() {

    }

    void PainterEngine::Draw() {

    }

    void PainterEngine::SetDrawArea(const Rect &area) {
        mArea = area;
    }

    void PainterEngine::SetPenColor(const Color &color) {
        mPalette.PenColor = color;
    }

    void PainterEngine::SetBrushColor(const Color &color) {
        mPalette.BrushColor = color;
    }

    void PainterEngine::SetFilling(bool fill) {
        mPalette.Filled = fill;
    }

    void PainterEngine::SetDrawPalette(const Palette &palette) {
        mPalette = palette;
    }

    const PainterEngine::Rect & PainterEngine::GetDrawArea() const {
        return mArea;
    }

    const PainterEngine::Palette & PainterEngine::GetDrawPalette() const {
        return mPalette;
    }

    const std::shared_ptr<Window> &PainterEngine::GetWindow() const {
        return mWindow;
    }

}