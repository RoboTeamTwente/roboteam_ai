//
// Created by mrlukasbos on 4-12-18.
//

#include "interface/api/Input.h"

namespace rtt {
namespace ai {
namespace interface {

// declare static variables
std::vector<Drawing> Input::drawings;
std::mutex Input::drawingMutex;
std::vector<TextDrawing> Input::textDrawings;
std::mutex Input::textDrawingMutex;
std::mutex Input::fpsMutex;
    std::mutex Input::cycleMutex;

int Input::FPS;
    int Input::index=0;

    QLineSeries * Input::cycleTime = new QLineSeries();


/*
 * Draw data to the screen
 */
void Input::drawData(Visual visual, std::vector<Vector2> points, QColor color, int robotId,
        Drawing::DrawingMethod method, double width, double height, double strokeWidth) {
    if (method == Drawing::DrawingMethod::ARROWS) {
        if (points.size() % 2 == 1) {
            points.erase(points.end());
        }
    }
    Input::makeDrawing(Drawing(visual, std::move(points), std::move(color), robotId, method, width, height, strokeWidth));
}

/*
 * Useful for debugging:  quickly draw a vector of points.
 */
void Input::drawDebugData(std::vector<Vector2> points, QColor color, int robotId, Drawing::DrawingMethod method, double width,
                          double height, double strokeWidth) {
    Input::makeDrawing(Drawing(Visual::DEBUG, std::move(points), std::move(color), robotId, method, width, height, strokeWidth));
}

void Input::makeDrawing(Drawing const &drawing) {
    std::lock_guard<std::mutex> lock(drawingMutex);
    drawings.push_back(drawing);
}

const std::vector<Drawing> Input::getDrawings() {
    std::lock_guard<std::mutex> lock(drawingMutex);
    return drawings;
}

const std::vector<TextDrawing> Input::getTextDrawings() {
    std::lock_guard<std::mutex> lock(textDrawingMutex);
    return textDrawings;
}

void Input::clearDrawings() {
    std::lock_guard<std::mutex> drawingLock(drawingMutex);
    textDrawings = {};

    std::lock_guard<std::mutex> textDrawingLock(textDrawingMutex);
    drawings = {};
}

Input::~Input() {
    clearDrawings();
}

void Input::drawText(Visual visual, QString text, QColor color, Vector2 location, int fontSize) {
    Input::makeTextDrawing(TextDrawing(visual, color, text, location, fontSize));
}

void Input::makeTextDrawing(TextDrawing const &textDrawing) {
    std::lock_guard<std::mutex> lock(textDrawingMutex);
    textDrawings.push_back(textDrawing);
}

int Input::getFps() {
    std::lock_guard<std::mutex> lock(fpsMutex);
    return FPS;
}

void Input::setFps(int fps) {
    std::lock_guard<std::mutex> lock(fpsMutex);
    FPS = fps;
}

QLineSeries * Input::getCycleTimes() {
    std::lock_guard<std::mutex> lock(cycleMutex);
    return cycleTime;
}

void Input::addCycleTime(const std::chrono::milliseconds &t) {
    std::lock_guard<std::mutex> lock(cycleMutex);

    cycleTime->setName("time");
    cycleTime->setUseOpenGL();
    cycleTime->setColor(Qt::yellow);
    std::chrono::milliseconds now = std::chrono::duration_cast< std::chrono::milliseconds >(
            std::chrono::system_clock::now().time_since_epoch());
    cycleTime->append(index++, t.count());
}


} // interface
} // ai
} // rtt