//
// Created by mrlukasbos on 19-9-18.
//

/*
 * Important note
 *
 *
 */
#ifndef ROBOTEAM_AI_IO_MANAGER_H
#define ROBOTEAM_AI_IO_MANAGER_H

#include "ros/ros.h"
#include "roboteam_msgs/World.h"
#include "roboteam_utils/constants.h"
#include "roboteam_utils/LastWorld.h"
#include <iostream>
#include <roboteam_msgs/GeometryData.h>

namespace rtt {
namespace ai {
namespace io {

class IOManager {
    private:
        roboteam_msgs::World world;
        roboteam_msgs::GeometryData geometry;
        ros::Subscriber worldSubscriber;
        ros::Subscriber geometrySubscriber;

    protected:
        ros::NodeHandle nodeHandle;

        void handleWorldState(const roboteam_msgs::WorldConstPtr &world);

        void handleGeometryData(const roboteam_msgs::GeometryDataConstPtr &geometry);

    public:
        IOManager() = default;

        void subscribeToWorldState();

        void subscribeToGeometryData();

        const roboteam_msgs::World &getWorldState();

        const roboteam_msgs::GeometryData &getGeometryData();
};

} // io
} // ai
} // rtt

#endif //ROBOTEAM_AI_IO_MANAGER_H