//
// Created by mrlukasbos on 19-9-18.
//

#include "IOManager.h"

namespace rtt {
namespace ai {
namespace io {

void IOManager::subscribeToWorldState() {
  worldSubscriber = nodeHandle.subscribe<roboteam_msgs::World>(rtt::TOPIC_WORLD_STATE, 1, &IOManager::handleWorldState, this);
}

void IOManager::subscribeToGeometryData() {
  geometrySubscriber = nodeHandle.subscribe<roboteam_msgs::GeometryData>(rtt::TOPIC_GEOMETRY, 1, &IOManager::handleGeometryData, this);
}

void IOManager::subscribeToRefereeData() {
  refereeSubscriber=nodeHandle.subscribe<roboteam_msgs::RefereeData>(rtt::TOPIC_REFEREE, 1, &IOManager::handleRefereeData,this);
}

void IOManager::handleWorldState(const roboteam_msgs::WorldConstPtr& world) {
  this->world = * world;
}

void IOManager::handleGeometryData(const roboteam_msgs::GeometryDataConstPtr &geometry) {
  this->geometry = * geometry;
}

void IOManager::handleRefereeData(const roboteam_msgs::RefereeDataConstPtr &refData) {
  this->refData = * refData;
}

const roboteam_msgs::World& IOManager::getWorldState() {
  return this->world;
}

const roboteam_msgs::GeometryData& IOManager::getGeometryData() {
  return this->geometry;
}

const roboteam_msgs::RefereeData& IOManager::getRefereeData() {
  return this->refData;
}

} // io
} // ai
} // rtt



