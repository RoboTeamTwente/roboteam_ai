//
// Created by rolf on 23-10-18.
//

#include "Referee.hpp"

namespace rtt {
namespace ai {

// Initialize static variables
roboteam_msgs::RefereeData Referee::refMsg;
roboteam_msgs::RefereeData Referee::previousRefMsg;

roboteam_msgs::RefereeData Referee::getRefereeData() {
    return Referee::refMsg;
}

void Referee::setRefereeData(roboteam_msgs::RefereeData refMsg) {
    Referee::previousRefMsg = Referee::refMsg; // set the old message as previous message
    Referee::refMsg = refMsg;
}
roboteam_msgs::RefereeData Referee::getPreviousRefereeData() {
    return Referee::previousRefMsg;
}

}//ai
}//rtt