//
// Created by mrlukasbos on 7-5-19.
//

#ifndef ROBOTEAM_AI_TOGGLES_H
#define ROBOTEAM_AI_TOGGLES_H

#include <map>
#include <QtCore/QString>
#include <vector>

namespace rtt {
namespace ai {
namespace interface {

enum Visual {
    DEBUG,
    BALL_DATA,
    PATHFINDING,
    PATHFINDING_DEBUG,
    KEEPER,
    INTERCEPT,
    DEFENSE,
    OFFENSE,
    SHOTLINES
};

enum showType {NO_ROBOTS = 0, SELECTED_ROBOTS = 1, ALL_ROBOTS = 2};
struct Toggle {
    Visual vis;
    showType defaultShowType;
    QString title;
};

struct Toggles {
    static std::vector<Toggle> toggles;
};

} // interface
} // ai
} // rtt

#endif //ROBOTEAM_AI_TOGGLES_H
