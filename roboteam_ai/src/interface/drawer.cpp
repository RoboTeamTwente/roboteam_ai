//
// Created by mrlukasbos on 4-12-18.
//

#include "drawer.h"

namespace rtt {
namespace ai {
namespace interface {

// declare static variables
std::vector<Vector2> Drawer::GoToPosLuThPoints;

void Drawer::setGoToPosLuThPoints(std::vector<rtt::Vector2> points) {
    Drawer::GoToPosLuThPoints = std::move(points);
}

const std::vector<Vector2> &Drawer::getGoToPosLuThPoints() {
    return GoToPosLuThPoints;
}

}
}
}