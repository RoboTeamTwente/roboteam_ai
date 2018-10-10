//
// Created by mrlukasbos on 10-10-18.
//

#ifndef ROBOTEAM_AI_CONDITION_H
#define ROBOTEAM_AI_CONDITION_H

/**
 * \class Condition
 * \brief Base class for conditions.
 */
class Condition : public Leaf {
 public:
  Condition(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
  virtual ~Condition();
  virtual Status Update();
};

#endif //ROBOTEAM_AI_CONDITION_H
