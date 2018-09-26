#pragma once

#include "../Composite.hpp"

namespace bt {

/*
    The Sequence composite ticks each child node in order.
    If a child fails or runs, the sequence returns the same status.
    In the next tick, it will try to run each child in order again.
    If all children succeeds, only then does the sequence succeed.
*/
class Sequence : public Composite {
 public:
  Status Update() override;

  std::string node_name() override;

  using Ptr = std::shared_ptr<Sequence>;
};

Sequence::Ptr MakeSequence();

}
