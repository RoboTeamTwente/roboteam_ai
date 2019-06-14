//
// Created by mrlukasbos on 4-12-18.
//

#ifndef ROBOTEAM_AI_INPUT_H
#define ROBOTEAM_AI_INPUT_H

#include <QtGui/QColor>
#include <roboteam_utils/Vector2.h>
#include <iostream>
#include <mutex>
#include <tuple>
#include "Toggles.h"

namespace rtt {
namespace ai {
namespace interface {

/*
 * For drawing to the interface we keep 'drawings' to draw data to the screen.
 * a drawing represents a vector of points and some specifications on how to display those.
 * e.g: form, color, size, and depth.
 */
struct Drawing {
    enum DrawingMethod {
      LINES_CONNECTED,
      DOTS,
      CROSSES,
      CIRCLES,
      PLUSSES
    };

    Drawing(Visual visual, std::vector<Vector2> points, QColor color, int robotId = -1, DrawingMethod method = DOTS, double width = 0.0, double height = 0.0, double strokeWidth = 0.0)
            : visual(visual),
            points(std::move(points)),
            color(std::move(color)),
            robotId(robotId),
            method(method),
            width(width),
            height(height),
            strokeWidth(strokeWidth){};

    Visual visual;
    std::vector<Vector2> points;
    QColor color;
    int robotId;
    DrawingMethod method;

    // these values are used for dots, crosses and circles
    double width = 4.0;
    double height = 4.0;
    double strokeWidth = 2.0;
};


class Input {
public:
    explicit Input() = default;

    virtual ~Input();

    static void clearDrawings();
    static const std::vector<Drawing> getDrawings();
    static void drawData(Visual visual, std::vector<Vector2> points, QColor color, int robotId = -1, Drawing::DrawingMethod method = Drawing::DOTS, double width = 4.0, double height = 4.0, double strokeWidth = 2.0);
    static void drawDebugData(std::vector<Vector2> points, QColor color = Qt::yellow, int robotId = -1, Drawing::DrawingMethod method = Drawing::DOTS, double width = 4.0, double height = 4.0, double strokeWidth = 4.0);

private:
    static std::vector<Drawing> drawings;
    static std::mutex drawingMutex;
    static void makeDrawing(Drawing const &drawing);
};

} // interface
} // ai
} // rtt

#endif //ROBOTEAM_AI_INPUT_H
