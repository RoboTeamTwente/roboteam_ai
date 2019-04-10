//
// Created by mrlukasbos on 4-12-18.
//

#ifndef ROBOTEAM_AI_DRAWER_H
#define ROBOTEAM_AI_DRAWER_H

#include <QtGui/QColor>
#include <roboteam_utils/Vector2.h>
#include <iostream>
#include <mutex>
#include <tuple>

namespace rtt {
namespace ai {
namespace interface {

class Drawer {
    public:
        explicit Drawer() = default;
        using GTPPoints = std::vector<std::pair<Vector2, QColor>>;
        static void setNumTreePoints(int id, GTPPoints points);
        static void addNumTreePoints(int id, GTPPoints points);
        static GTPPoints getNumTreePoints(int id);
        static void setKeeperPoints(int id, GTPPoints points);
        static GTPPoints getKeeperPoints(int id);
        static void setInterceptPoints(int id, GTPPoints points);
        static GTPPoints getInterceptPoints(int id);

    static GTPPoints getAttackerPoints(int id);

        static std::vector<std::pair<Vector2, QColor>> getDrawPoints();
        static std::vector<std::tuple<Vector2, Vector2, QColor>> getDrawLines();
        static void clearDrawLines();
        static void clearDrawPoints();

private:
        static std::mutex drawMutex,goToPosMutex,keeperMutex,interceptMutex,offensiveMutex,attackerMutex,drawLinesMutex;
        static std::map<int, GTPPoints> GoToPosLuThPoints;


        static void drawPoint(Vector2 point, QColor color = Qt::darkMagenta);
        static void drawPoint(std::pair<Vector2, QColor> point);
        static void drawPoints(std::vector<std::pair<Vector2, QColor>> points);

        static void drawLine(Vector2 pointA, Vector2 pointB, QColor color = Qt::darkMagenta);
        static std::map<int, GTPPoints> NumTreePoints;
        static std::map<int, GTPPoints> KeeperPoints;
        static std::map<int, GTPPoints> InterceptPoints;
        static std::map<int, GTPPoints> AttackerPoints;
        static GTPPoints OffensivePoints;

        static void addDrawPoint(Vector2 point, QColor color = Qt::darkMagenta);
        static void addDrawPoint(std::pair<Vector2, QColor> point);
        static void addDrawPoints(std::vector<std::pair<Vector2, QColor>> points);

        static std::vector<std::pair<Vector2, QColor>> drawP;
        static std::vector<std::tuple<Vector2, Vector2, QColor>> drawL;

};

} // interface
} // ai
} // rtt
#endif //ROBOTEAM_AI_DRAWER_H
