//
// Created by timovdk on 5/20/20.
/// TODO Needs to be refactored to use new passComputations (or play should be removed)
//

#include "stp/plays/offensive/GenericPass.h"

#include <roboteam_utils/Tube.h>

#include "stp/computations/PositionComputations.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/active/PassReceiver.h"
#include "stp/roles/active/Passer.h"
#include "stp/roles/passive/BallDefender.h"
#include "stp/roles/passive/Formation.h"
#include "stp/roles/passive/Halt.h"

namespace rtt::ai::stp::play {}  // namespace rtt::ai::stp::play
