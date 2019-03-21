//
// Created by thijs on 19-3-19.
//

#ifndef ROBOTEAM_AI_PREDICTEDWORLD_H
#define ROBOTEAM_AI_PREDICTEDWORLD_H

#include "LastWorld.h"

namespace rtt {
namespace ai {
namespace world {

class PredictedWorld : public LastWorld {
    private:

    public:
        void updatePredictedWorld() override;

};

}
}
}

#endif //ROBOTEAM_AI_PREDICTEDWORLD_H
