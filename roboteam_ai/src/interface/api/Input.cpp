//
// Created by mrlukasbos on 4-12-18.
//

#include "Input.h"

namespace rtt {
namespace ai {
namespace interface {

// declare static variables
std::vector<Drawing> Input::drawings;
std::mutex Input::drawingMutex;

/*
 * Draw data to the screen
 */
void Input::drawData(std::string const &name, std::vector<Vector2> points, QColor color, int robotId,
        Drawing::DrawingMethod method, Drawing::Depth depth, double width, double height, double strokeWidth) {
    Input::makeDrawing(Drawing(name, std::move(points), std::move(color), robotId, method, depth, width, height, strokeWidth));
}

/*
 * Useful for debugging:  quickly draw a vector of points.
 */
void Input::drawDebugData(std::vector<Vector2> points, QColor color, int robotId, Drawing::DrawingMethod method, double width,
                          double height, double strokeWidth) {
    Input::makeDrawing(Drawing("debug", std::move(points), std::move(color), robotId, method, Drawing::FRONT, width, height, strokeWidth));
}

void Input::makeDrawing(Drawing const &drawing) {
    std::lock_guard<std::mutex> lock(drawingMutex);
    drawings.push_back(drawing);
}

const std::vector<Drawing> &Input::getDrawings() {
    std::lock_guard<std::mutex> lock(drawingMutex);
    return drawings;
}

void Input::clearDrawings() {
    std::lock_guard<std::mutex> lock(drawingMutex);
    drawings = {};
}



} // interface
} // ai
} // rtt