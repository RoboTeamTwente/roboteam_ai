//
// Created by kjhertenberg on 16-5-19.
//

#ifndef ROBOTEAM_WORLD_KALMANUS_H
#define ROBOTEAM_WORLD_KALMANUS_H

#include "kalmanObject.h"

namespace rtt {

class kalmanUs : public kalmanObject {
    //in the future data about them and us might be different so we made different classes
    public:

        kalmanUs();

        kalmanUs(uint id);

};
}

#endif //ROBOTEAM_WORLD_KALMANUS_H
