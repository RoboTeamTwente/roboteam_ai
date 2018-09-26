//
// Created by mrlukasbos on 21-9-18.
//
// Start a strategy node

#include "ros/ros.h"
#include "io/IO_Node.h"

#define ROS_LOG_NAME "StrategyNode"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "StrategyNode");
  ros::Rate rate(10);

  IO_Node rosHandler;
  rosHandler.subscribeAll();

  ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "Initializing StrategyNode " << ros::this_node::getName());


  // Construct the global role directive publisher & bt debug publisher if needed
  rtt::GlobalPublisher<roboteam_msgs::RoleDirective> globalRoleDirectivePublisher(rtt::TOPIC_ROLE_DIRECTIVE);
  CREATE_GLOBAL_RQT_BT_TRACE_PUBLISHER;

  // Creates the callbacks and removes them at the end
  rtt::WorldAndGeomCallbackCreator cb;
  ROS_INFO_NAMED(ROS_LOG_NAME, "Waiting for first world & geom message...");
  rtt::LastWorld::wait_for_first_messages();

  std::vector<std::string> arguments(argv + 1, argv + argc);

  // Subscribe to referee
  ros::Subscriber ref_sub = n.subscribe<roboteam_msgs::RefereeData>("vision_refbox", 1000, refereeCallback);
  ROS_INFO_NAMED(ROS_LOG_NAME, "Subscribed to 'vision_refbox'");

  StrategyDebugInfo stratDebugInfo;

  std::shared_ptr<bt::BehaviorTree> strategy;

  // Only continue if arguments were given
  if (arguments.size() > 0) {
    if (arguments[0]=="mainStrategy") {
      strategy = rtt::StrategyComposer::getMainStrategy();

      ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "mainStrategy: " << strategy);
      if (!strategy) {
        ROS_ERROR_NAMED(ROS_LOG_NAME,
                        "There was an error initializing the main strategy tree (StrategyComposer). Aborting.");
        return 1;
      }

      if (!rtt::LastRef::waitForFirstRefCommand()) {
        return 0;
      }
    } else {
      // Get all available trees
      namespace f = rtt::factories;
      auto &repo = f::getRepo < f::Factory < bt::BehaviorTree >> ();
      // If the given name exists...


      std::cout << "[StrategyNode] Printing all repos.." << std::endl;
      for (const auto &entry : repo) {
        std::cout << "\t- " << entry.first << "\n";
      }

      if (repo.find(arguments[0])!=repo.end()) {
        // Get the factory
        auto treeFactory = repo.at(arguments[0]);
        // Create the tree
        strategy = treeFactory("", nullptr);

        roboteam_msgs::Blackboard bb;

        RTT_SEND_RQT_BT_TRACE(
            roboteam_msgs::BtDebugInfo::ID_STRATEGY_NODE,
            arguments[0],
            roboteam_msgs::BtDebugInfo::TYPE_STRATEGY,
            roboteam_msgs::BtStatus::STARTUP,
            bb
        );

      } else {
        ROS_ERROR_NAMED(ROS_LOG_NAME, "\"%s\" is not a strategy tree. Aborting.", arguments[0].c_str());
        return 1;
      }
    }

  } else {
    ROS_ERROR_NAMED(ROS_LOG_NAME, "No strategy tree passed as argument. Aborting.");
    return 1;
  }

  ROS_INFO_STREAM_NAMED(ROS_LOG_NAME, "Strategy set!");

  // Wait for all the role nodes to come online if a param was set
  if (rtt::has_PARAM_NUM_ROLE_NODES()) {
    int numNodes;
    rtt::get_PARAM_NUM_ROLE_NODES(numNodes);

    ROS_DEBUG_NAMED(ROS_LOG_NAME, "Waiting for %i robot nodes to come online", numNodes);

    auto directivePub = n.advertise<roboteam_msgs::RoleDirective>(rtt::TOPIC_ROLE_DIRECTIVE, 100);
    while ((int) directivePub.getNumSubscribers() < numNodes) {
      ROS_DEBUG_NAMED(ROS_LOG_NAME, "Current num subsribers: %i", (int) directivePub.getNumSubscribers());
      ros::spinOnce();
      rate.sleep();

      if (!ros::ok()) {
        RTT_DEBUGLN("Interrupt received, exiting...");
        return 0;
      }
    }
  }

  ROS_DEBUG_NAMED(ROS_LOG_NAME, "Found role nodes. Waiting for more than 0 robots to appear...");

  while (rtt::LastWorld::get().us.size()==0) {
    ros::spinOnce();
    rate.sleep();

    if (!ros::ok()) {
      ROS_DEBUG_NAMED(ROS_LOG_NAME, "Interrupt received, exiting...");
      return 0;
    }
  }

  // This is overwritten as soon as the first RefereeCommand comes in, which has the actual keeper robot ID in it.
  rtt::RobotDealer::setKeeper(0);

  z(ROS_LOG_NAME, "More than one robot found. Starting!");

  if (!strategy) {
    ROS_FATAL_NAMED(ROS_LOG_NAME, "No strategy tree loaded! Terminating...");
    exit(-1);
  }

  while (ros::ok()) {
    ros::spinOnce();

    bt::Node::Status status = strategy->Update();

    if (status!=bt::Node::Status::Running) {
      auto statusStr = bt::statusToString(status);
      ROS_DEBUG_STREAM_NAMED(ROS_LOG_NAME, "Strategy result: " << statusStr.c_str() << "Shutting down...\n");
      break;
    }
    rate.sleep();

    // Update the keeper according to the ref info
    if (rtt::LastRef::hasReceivedFirstCommand()) {
      rtt::RobotDealer::setKeeper(rtt::LastRef::get().us.goalie);
    }

    stratDebugInfo.doUpdate(strategy);
  }

  // Terminate if needed
  if (strategy->getStatus()==bt::Node::Status::Running) {
    strategy->Terminate(bt::Node::Status::Running);
  }

  return 0;
}
