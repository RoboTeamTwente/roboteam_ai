//
// Created by mrlukasbos on 7-6-19.
//

#ifndef ROBOTEAM_AI_PASS_H
#define ROBOTEAM_AI_PASS_H

#include <roboteam_ai/src/world/Robot.h>
#include <roboteam_utils/Line.h>
#include <roboteam_ai/src/coach/heuristics/PassScore.h>

namespace rtt {
namespace ai {

class Pass {
private:
    const double MAX_PASS_TIME = 5.0;
    std::shared_ptr<world::Robot> passer;
    std::shared_ptr<world::Robot> receiver;
    Vector2 passEnd;
    bool locationBasedPass = false;
    bool ballShouldLayStill = false;

    // timing of the pass
    bool passKicked = false;
    bool receiverReady = false;
    ros::Time passKickedTime;
    ros::Time passInitializedTime;


public:
    // constructors
    explicit Pass() = default;
    explicit Pass(std::shared_ptr<world::Robot> passer, std::shared_ptr<world::Robot> receiver);
    explicit Pass(std::shared_ptr<world::Robot> passer,  std::shared_ptr<world::Robot> receiver, const Vector2& endPos, bool ballShouldLayStill = true);

    // helpers
    bool passTakesTooLong();
    bool passFailed();
    bool passSucceeded();

    // getters & setters
    Line getPassLine();
    bool isPassKicked() const;
    void setPassKicked(bool kicked);
    bool isBallShouldLayStill() const;
    void setBallShouldLayStill(bool shouldLayStill);
    bool isReceiverReady() const;
    void setReceiverReady(bool receiverReady);
    const Vector2 &getPassEnd() const;
    double getScore();

};

}
}

#endif //ROBOTEAM_AI_PASS_H
