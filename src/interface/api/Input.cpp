//
// Created by mrlukasbos on 4-12-18.
//

#include "interface/api/Input.h"

namespace rtt::ai::interface {

// declare static variables
std::vector<Drawing> Input::drawings;
std::mutex Input::drawingMutex;
std::mutex Input::fpsMutex;

int Input::FPS;

/*
 * Draw data to the screen
 */
void Input::drawData(Visual visual, std::vector<Vector2> points, QColor color, int robotId, Drawing::DrawingMethod method, double width, double height, double strokeWidth) {
    if (method == Drawing::DrawingMethod::ARROWS) {
        if (points.size() % 2 == 1) {
            points.erase(points.end());
        }
    }
    std::lock_guard<std::mutex> lock(drawingMutex);
    drawings.emplace_back(visual, std::move(points), std::move(color), robotId, method, width, height, strokeWidth);
}

/*
 * Useful for debugging:  quickly draw a vector of points.
 */
void Input::drawDebugData(std::vector<Vector2> points, QColor color, int robotId, Drawing::DrawingMethod method, double width, double height, double strokeWidth) {
    std::lock_guard<std::mutex> lock(drawingMutex);
    drawings.emplace_back(Visual::DEBUG, std::move(points), std::move(color), robotId, method, width, height, strokeWidth);
}

const std::vector<Drawing> Input::getDrawings() {
    std::lock_guard<std::mutex> lock(drawingMutex);
    return drawings;
}

void Input::clearDrawings() {
    std::lock_guard<std::mutex> drawingLock(drawingMutex);
    drawings.clear();
}

Input::~Input() { clearDrawings(); }

int Input::getFps() {
    std::lock_guard<std::mutex> lock(fpsMutex);
    return FPS;
}

void Input::setFps(int fps) {
    std::lock_guard<std::mutex> lock(fpsMutex);
    FPS = fps;
}

Input::Input() { drawings.reserve(10000); }

}  // namespace rtt::ai::interface