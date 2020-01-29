//
// Created by mrlukasbos on 7-5-19.
//

#ifndef ROBOTEAM_AI_TOGGLES_H
#define ROBOTEAM_AI_TOGGLES_H

#include <QtCore/QString>
#include <map>
#include <vector>

namespace rtt::ai::interface {

enum Visual { DEBUG, BALL_DATA, BALL_HANDLING, PATHFINDING, PATHFINDING_DEBUG, KEEPER, INTERCEPT, DEFENSE, OFFENSE, SHOTLINES, BALLPLACEMENT, AVOIDANCE };

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
    explicit Toggle(Visual vis, RobotShowType rst, QString t) : visual(vis), robotShowType(rst), title(t) { showType = ROBOT; };
    explicit Toggle(Visual vis, GeneralShowType gst, QString t) : visual(vis), generalShowType(gst), title(t) { showType = GENERAL; };
};

class Toggles {
   public:
    static std::vector<Toggle> toggles;
};

}  // namespace rtt::ai::interface

#endif  // ROBOTEAM_AI_TOGGLES_H
