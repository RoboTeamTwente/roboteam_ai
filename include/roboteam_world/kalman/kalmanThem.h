//
// Created by kjhertenberg on 16-5-19.
//

#ifndef ROBOTEAM_WORLD_KALMANTHEM_H
#define ROBOTEAM_WORLD_KALMANTHEM_H

#include "kalmanObject.h"

namespace rtt {

    class kalmanThem : public kalmanObject{
    //in the future data about them and us might be different so we made different classes
    public:

        kalmanThem();

        kalmanThem(uint id);
    };

}

#endif //ROBOTEAM_WORLD_KALMANTHEM_H
