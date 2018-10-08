#pragma once

#include "DangerModule.h"

namespace rtt {
namespace ai {
namespace dangerfinder {
/**
 * \class FreeModule
 * \brief DangerModule which checks whether a robot could potentially receive the ball.
 * If so, it gives it a score and sets the DANGER_FREE flag.
 */

class FreeModule : public DangerModule {
 public:
  explicit FreeModule() = default;
  explicit FreeModule(double danger);
  PartialResult calculate(const roboteam_msgs::WorldRobot& bot, const roboteam_msgs::World& world) override;
};

} // dangerfinder
} // ai
} // rtt
