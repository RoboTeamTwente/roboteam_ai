//
// Created by mrlukasbos on 7-6-19.
//

#ifndef ROBOTEAM_AI_PASS_H
#define ROBOTEAM_AI_PASS_H

#include <roboteam_ai/src/world/Robot.h>
#include <roboteam_utils/Line.h>

namespace rtt {
namespace ai {

class Pass {
private:
    std::shared_ptr<world::Robot> passer;
    std::shared_ptr<world::Robot> receiver;
    Vector2 passEnd;
    bool ballShouldLayStill = false;

    // timing of the pass
    bool passKicked = false;
    ros::Time passKickedTime;

public:
    // constructors
    explicit Pass() = default;
    explicit Pass(std::shared_ptr<world::Robot> passer, std::shared_ptr<world::Robot> receiver);
    explicit Pass(std::shared_ptr<world::Robot> passer,  std::shared_ptr<world::Robot> receiver, Vector2 endPos, bool ballShouldLayStill = true);

    // getters & setters
    Line getPassLine();
    bool isPassKicked() const;
    void setPassKicked(bool kicked);
    bool isBallShouldLayStill() const;
    void setBallShouldLayStill(bool shouldLayStill);
};

}
}

#endif //ROBOTEAM_AI_PASS_H
