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

enum ShowType { GENERAL, ROBOT };
enum RobotShowType { NO_ROBOTS = 0, SELECTED_ROBOTS = 1, ALL_ROBOTS = 2 };
enum GeneralShowType { OFF = 0, ON = 1 };

class Toggle {
    public:
        Visual visual;
        ShowType showType;
        RobotShowType robotShowType;
        GeneralShowType generalShowType;
        QString title;

        Toggle() = default;
        explicit Toggle(Visual vis, ShowType st, RobotShowType rst, QString t)
                : visual(vis), showType(st), robotShowType(rst), title(t) {
            showType = ROBOT;
        };
        explicit Toggle(Visual vis, ShowType st, GeneralShowType gst, QString t)
                : visual(vis), showType(st), generalShowType(gst), title(t) {
            showType = GENERAL;
        };
};

class Toggles {
    public:
        static std::vector<Toggle> toggles;
};

} // interface
} // ai
} // rtt

#endif //ROBOTEAM_AI_TOGGLES_H
