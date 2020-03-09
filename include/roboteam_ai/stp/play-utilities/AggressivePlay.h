//
// Created by jessevw on 13.02.20.
//

#ifndef RTT_AGRESSIVEPLAY_H
#define RTT_AGRESSIVEPLAY_H

#include "Play.h"
namespace rtt::ai::analysis {
class AggressivePlay : public Play {
    AggressivePlay();
    void calculateInformation();
};
}  // namespace rtt::ai::analysis

#endif  // RTT_AGRESSIVEPLAY_H
