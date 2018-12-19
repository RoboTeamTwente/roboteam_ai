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
        static void setGoToPosLuThPoints(int id, std::vector<std::pair<Vector2, QColor>> points);
        static std::vector<std::pair<Vector2, QColor>> getGoToPosLuThPoints(int id);

    private:
        static std::mutex mutex;
        static std::map<int, std::vector<std::pair<Vector2, QColor>>> GoToPosLuThPoints;
};

} // interface
} // ai
} // rtt
#endif //ROBOTEAM_AI_DRAWER_H
