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

void Input::clearDrawings() {
    std::lock_guard<std::mutex> lock(drawingMutex);
    drawings = {};
}

Input::~Input() {
    clearDrawings();
}


} // interface
} // ai
} // rtt