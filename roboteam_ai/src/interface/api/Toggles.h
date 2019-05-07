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
    PATHFINDING,
    KEEPER,
    INTERCEPT,
    DEFENSE,
    SHOTLINES
};

enum showType {NO_ROBOTS = 0, SELECTED_ROBOTS = 1, ALL_ROBOTS = 2};
struct Toggle {
    Visual vis;
    showType defaultShowType;
    QString title;
};

static std::vector<Toggle> toggles = {
        {Visual::DEBUG,         showType::ALL_ROBOTS,       "Show debug values"},
        {Visual::PATHFINDING,   showType::SELECTED_ROBOTS,  "Show pathfinding"},
        {Visual::KEEPER,        showType::NO_ROBOTS,       "Show keeper points"},
        {Visual::INTERCEPT,     showType::ALL_ROBOTS,       "Show Interceptions"},
        {Visual::DEFENSE,       showType::ALL_ROBOTS,       "Show defensive location"},
        {Visual::SHOTLINES,       showType::ALL_ROBOTS,       "Show desired shotlines"},

};

} // interface
} // ai
} // rtt

#endif //ROBOTEAM_AI_TOGGLES_H
