//
// Created by mrlukasbos on 4-12-18.
//

#ifndef ROBOTEAM_AI_DRAWER_H
#define ROBOTEAM_AI_DRAWER_H

#include <QtGui/QColor>
#include <roboteam_utils/Vector2.h>
#include <iostream>
#include <mutex>

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

        static void addDrawPoint(Vector2 point, QColor color = Qt::darkMagenta);
        static void addDrawPoint(std::pair<Vector2, QColor> point);
        static void addDrawPoints(std::vector<std::pair<Vector2, QColor>> points);
        static std::vector<std::pair<Vector2, QColor>> getDrawPoints();
    private:
        static std::mutex goToPosMutex,keeperMutex,interceptMutex, drawMutex;
        static std::map<int, GTPPoints> NumTreePoints;
        static std::map<int, GTPPoints> KeeperPoints;
        static std::map<int, GTPPoints> InterceptPoints;
        static std::vector<std::pair<Vector2, QColor>> drawPoints;

};

} // interface
} // ai
} // rtt
#endif //ROBOTEAM_AI_DRAWER_H
