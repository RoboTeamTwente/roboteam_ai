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
#include <QtCharts/QtCharts>

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
      PLUSSES,
      ARROWS,
      REAL_LIFE_DOTS,
      REAL_LIFE_CIRCLES
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

struct TextDrawing {

    TextDrawing(  Visual visual, QColor color, QString text, Vector2 location, int fontSize)
    : visual(visual),
    color(color),
    text(text),
    location(location),
    fontSize(fontSize){};

    Visual visual;
    QColor color;
    QString text;
    Vector2 location;
    int fontSize;

};


class Input {
public:
    explicit Input() = default;
    virtual ~Input();

    static void clearDrawings();
    static const std::vector<Drawing> getDrawings();
    static void drawData(Visual visual, std::vector<Vector2> points, QColor color, int robotId = -1, Drawing::DrawingMethod method = Drawing::DOTS, double width = 4.0, double height = 4.0, double strokeWidth = 2.0);
    static void drawDebugData(std::vector<Vector2> points, QColor color = Qt::yellow, int robotId = -1, Drawing::DrawingMethod method = Drawing::DOTS, double width = 4.0, double height = 4.0, double strokeWidth = 4.0);
    static int getFps();
    static void setFps(int fps);
private:
    static std::vector<Drawing> drawings;
    static std::mutex drawingMutex;
    static std::mutex fpsMutex;
    static void makeDrawing(Drawing const &drawing);
    static int FPS;
};

} // interface
} // ai
} // rtt

#endif //ROBOTEAM_AI_INPUT_H
