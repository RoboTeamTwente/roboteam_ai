//
// Created by mrlukasbos on 4-12-18.
//

#ifndef ROBOTEAM_AI_DRAWER_H
#define ROBOTEAM_AI_DRAWER_H


#include <roboteam_utils/Vector2.h>
#include <iostream>

namespace rtt {
namespace ai {
namespace interface {

class Drawer {
public:
    explicit Drawer() = default;

    static void setGoToPosLuThPoints(int id, std::vector<Vector2> points);
    static std::vector<Vector2> getGoToPosLuThPoints(int id);

private:
    static std::map<int, std::vector<Vector2>> GoToPosLuThPoints;
};

} // interface
} // ai
} // rtt
#endif //ROBOTEAM_AI_DRAWER_H
