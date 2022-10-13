//
// Created by jordi on 19-05-20.
//

#include "stp/plays/ReflectKick.h"

#include "stp/evaluations/game_states/NormalOrFreeKickUsGameStateEvaluation.h"
#include "stp/evaluations/global/BallCloseToUsGlobalEvaluation.h"
#include "stp/evaluations/global/BallClosestToUsGlobalEvaluation.h"
#include "stp/evaluations/global/WeHaveBallGlobalEvaluation.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/active/BallReflector.h"
#include "stp/roles/active/Passer.h"
#include "stp/roles/passive/BallDefender.h"
#include "stp/roles/passive/Formation.h"

namespace rtt::ai::stp::play {}  // namespace rtt::ai::stp::play
