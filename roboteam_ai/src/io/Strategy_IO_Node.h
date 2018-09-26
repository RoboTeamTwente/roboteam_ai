//
// Created by mrlukasbos on 24-9-18.
//

#ifndef ROBOTEAM_AI_STRATEGY_IO_NODE_H
#define ROBOTEAM_AI_STRATEGY_IO_NODE_H

#include "IO_Node.h"

class Strategy_IO_Node : public IO_Node {
 public:
  Strategy_IO_Node();
  void subscribeToRoleFeedback();
  void publishRoleDirective();
};

#endif //ROBOTEAM_AI_STRATEGY_IO_NODE_H
