//
// Created by mrlukasbos on 24-9-18.
//

#ifndef ROBOTEAM_AI_ROLE_IO_NODE_H
#define ROBOTEAM_AI_ROLE_IO_NODE_H

#include "IO_Node.h"

class Role_IO_Node : public IO_Node {
 public:
  Role_IO_Node();

 private:
  void handleRobotFeedback(const roboteam_msgs::RoleFeedbackConstPtr &str);
};

#endif //ROBOTEAM_AI_ROLE_IO_NODE_H
