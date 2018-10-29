
#include "ros/ros.h"
#include "io/RoleIOManager.h"

namespace io = rtt::ai::io;

roboteam_msgs::World worldMsg;

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "RoleNode");

    // create a nodehandle and prepare subscriptions
    io::RoleIOManager roleIOManager;

    //
    roleIOManager.subscribeToWorldState();
    roleIOManager.subscribeToGeometryData();
    roleIOManager.subscribeToRoleDirective();

    ros::Rate rate(10);


    while (ros::ok()) {
        ros::spinOnce();
        worldMsg = roleIOManager.getWorldState();
        roleIOManager.getRoleDirective();


        rate.sleep();
    }
    return 0;
}
