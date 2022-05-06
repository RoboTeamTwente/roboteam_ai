#pragma once

#include <roboteam_utils/AIData.hpp>

/* This static class collects data that has to be sent to the interface.
 * The data will be stored in the AIData struct. */

namespace rtt {

class InterfaceData {
   public:
    static void clearData();
    static const AIData& getData();

    static void addRobotPath(const RobotPath& path);
    static void addSTPStatus(const RobotSTP& status);

};

} // namespace rtt