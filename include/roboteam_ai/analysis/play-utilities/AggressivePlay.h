//
// Created by jessevw on 13.02.20.
//

#ifndef RTT_AGRESSIVEPLAY_H
#define RTT_AGRESSIVEPLAY_H

#include "analysis/play-utilities/Play.h"
namespace rtt::ai::analysis {
    class AggressivePlay : public Play {
        AggressivePlay();
        void calculateInformation();
    };
}

#endif //RTT_AGRESSIVEPLAY_H
