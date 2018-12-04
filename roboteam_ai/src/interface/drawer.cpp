//
// Created by mrlukasbos on 4-12-18.
//

#include "drawer.h"

namespace rtt {
namespace ai {
namespace interface {

// declare static variables
std::map<int, std::vector<Vector2>> Drawer::GoToPosLuThPoints;

void Drawer::setGoToPosLuThPoints(int id, std::vector<rtt::Vector2> points) {
    std::pair<int, std::vector<rtt::Vector2>> pair{id, std::move(points)};

     GoToPosLuThPoints.erase(id);
     Drawer::GoToPosLuThPoints.insert(pair);
}

std::vector<Vector2> Drawer::getGoToPosLuThPoints(int id) {

    if (GoToPosLuThPoints.find(id) != GoToPosLuThPoints.end()) {
        return GoToPosLuThPoints[id];
    }
    return {};
}

}
}
}