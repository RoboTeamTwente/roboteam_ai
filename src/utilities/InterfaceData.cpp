#include <utilities/InterfaceData.hpp>

namespace rtt {

AIData data = {};

void InterfaceData::clearData() {
    data = {};
}

const AIData &InterfaceData::getData() {
    return data;
}

void InterfaceData::addRobotPath(const RobotPath &path) {
    data.robotPaths.push_back(path);
}

void InterfaceData::addSTPStatus(const RobotSTP &status) {
    data.robotStps.push_back(status);
}

} // namespace rtt