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
        static void setGoToPosLuThPoints(int id, GTPPoints points);
        static void addGoToPosLuThPoints(int id, GTPPoints points);
        static GTPPoints getGoToPosLuThPoints(int id);
        static void setKeeperPoints(int id, GTPPoints points);
        static GTPPoints getKeeperPoints(int id);
        static void setInterceptPoints(int id, GTPPoints points);
        static GTPPoints getInterceptPoints(int id);
    private:
        static std::mutex goToPosMutex,keeperMutex,interceptMutex;
        static std::map<int, GTPPoints> GoToPosLuThPoints;
        static std::map<int, GTPPoints> KeeperPoints;
        static std::map<int, GTPPoints> InterceptPoints;
};

} // interface
} // ai
} // rtt
#endif //ROBOTEAM_AI_DRAWER_H
