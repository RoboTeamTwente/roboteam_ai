/*
 * returns SUCCESS if the ball is in the given defence area (standard ours) AND if the ball lays still
 * 
 */

#ifndef ROBOTEAM_AI_BALLINDEFENSEAREAANDSTILL_H
#define ROBOTEAM_AI_BALLINDEFENSEAREAANDSTILL_H

#include "Condition.h"

namespace rtt{
namespace ai{

class BallInDefenseAreaAndStill : public Condition{
private:
    bool theirDefenceArea;
public:
    explicit BallInDefenseAreaAndStill(std::string name = "BallInDefenseAreaAndStill", bt::Blackboard::Ptr blackboard = nullptr);
    void initialize() override;
    Status update() override;
    std::string node_name() override;
};

} // ai
} // rtt


#endif //ROBOTEAM_AI_BALLINDEFENSEAREAANDSTILL_H
