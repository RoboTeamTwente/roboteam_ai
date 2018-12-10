//
// Created by rolf on 10/12/18.
//

#ifndef ROBOTEAM_AI_KEEPER_H
#define ROBOTEAM_AI_KEEPER_H
#include "Skill.h"
#include "roboteam_utils/Arc.h"

namespace rtt{
namespace ai{
class Keeper : public Skill {
    private:
        Arc blockCircle;

        Vector2 computeBlockPoint(Vector2 defendPos);
        Vector2 goalPos;
    public:
        explicit Keeper(string name, bt::Blackboard::Ptr blackboard);
        std::string node_name() override;
        Status update() override;
        void initialize() override;
        void terminate(Status s) override;
};
}
}


#endif //ROBOTEAM_AI_KEEPER_H
