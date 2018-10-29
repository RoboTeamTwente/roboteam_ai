#pragma once

#include "../Composite.hpp"

namespace bt {

/*
    The Selector composite ticks each child node in order, and remembers what child it prevously tried to tick.
    If a child succeeds or runs, the sequence returns the same status.
    In the next tick, it will try to run each child in order again.
    If all children fails, only then does the selector fail.
*/
class MemSelector : public Composite {
 public:
  size_t index;

  void Initialize() override;

  Status Update() override;

  using Ptr = std::shared_ptr<MemSelector>;

  std::string node_name() override;
};
} // bt
