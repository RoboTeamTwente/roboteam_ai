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

namespace rtt {
namespace ai {
namespace interface {

/*
 * For drawing to the interface we keep 'drawings' to draw data to the screen.
 * a drawing represents a vector of points and some specifications on how to display those.
 * e.g: form, color, size, and depth.
 */
struct Drawing {
    enum DrawingMethod {LINES_SEPARATE, LINES_CONNECTED, DOTS, CROSSES, CIRCLES};
    enum Depth {FRONT, MIDDLE, BACK};

    Drawing(std::string const &name, std::vector<Vector2> points, QColor color, DrawingMethod method = DOTS, Depth depth = FRONT, double width = 0.0, double height = 0.0, double strokeWidth = 0.0)
            : name(QString::fromStdString(name)),
            points(std::move(points)),
            color(std::move(color)),
            method(method),
            depth(depth),
            width(width),
            height(height),
            strokeWidth(strokeWidth){};

    QString name;
    std::vector<Vector2> points;
    QColor color;
    DrawingMethod method;
    Depth depth;

    // these values are used for dots, crosses and circles
    double width = 4.0;
    double height = 4.0;
    double strokeWidth = 2.0;
};


class Input {
public:
    explicit Input() = default;
    static void clearDrawings();
    static const std::vector<Drawing> &getDrawings();
    static void drawData(std::string const &name, std::vector<Vector2> points, QColor color, Drawing::DrawingMethod method = Drawing::DOTS, Drawing::Depth depth = Drawing::FRONT, double width = 0.0, double height = 0.0, double strokeWidth = 0.0);
private:
    static std::vector<Drawing> drawings;
    static std::mutex drawingMutex;
    static void makeDrawing(Drawing const &drawing);
};

} // interface
} // ai
} // rtt

#endif //ROBOTEAM_AI_INPUT_H
