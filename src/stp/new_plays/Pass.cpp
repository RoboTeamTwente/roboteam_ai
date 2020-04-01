//
// Created by jessevw on 17.03.20.
//

#include "stp/new_plays/Pass.h"
#include <stp/new_roles/TestRole.h>
#include <stp/invariants/WeHaveBallInvariant.h>
#include <utility>
#include "stp/new_roles/PassReceiver.h"
#include "stp/new_roles/Passer.h"
#include "pagmo/algorithms/pso_gen.hpp"
#include "stp/new_plays_analysis/PassProblem.h"
#include "pagmo/population.hpp"

namespace rtt::ai::stp::play {

Pass::Pass(std::string playName) : Play(std::move(playName)) {
    invariants.clear();
    invariants.emplace_back(std::make_unique<invariant::WeHaveBallInvariant>());

    roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{
        std::make_unique<role::Passer>(role::Passer("passer")), std::make_unique<role::PassReceiver>(role::PassReceiver("pass_receiver")),
        std::make_unique<TestRole>(TestRole("defender1")),      std::make_unique<TestRole>(TestRole("defender2")),
        std::make_unique<TestRole>(TestRole("defender3")),    std::make_unique<TestRole>(TestRole("defender4")),
        std::make_unique<TestRole>(TestRole("defender5")),    std::make_unique<TestRole>(TestRole("defender6")),
        std::make_unique<TestRole>(TestRole("defender7")),    std::make_unique<TestRole>(TestRole("defender8")),
        std::make_unique<TestRole>(TestRole("defender9"))};
}

uint8_t Pass::score(world_new::World* world) noexcept { return 20; }

Dealer::FlagMap Pass::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag closeToBallFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag closeToTheirGoalFlag(DealerFlagTitle::CLOSE_TO_THEIR_GOAL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag notImportant(DealerFlagTitle::NOT_IMPORTANT, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"passer", {closeToBallFlag}});
    flagMap.insert({"pass_receiver", {closeToTheirGoalFlag}});
    flagMap.insert({"defender1", {notImportant}});
    flagMap.insert({"defender2", {notImportant}});
    flagMap.insert({"defender3", {notImportant}});
    flagMap.insert({"defender4", {notImportant}});
    flagMap.insert({"defender5", {notImportant}});
    flagMap.insert({"defender6", {notImportant}});
    flagMap.insert({"defender7", {notImportant}});
    flagMap.insert({"defender8", {notImportant}});
    flagMap.insert({"defender9", {notImportant}});

    return flagMap;
}

void Pass::calculateInfoForRoles() noexcept {
    // Calculate most important positions to defend
    // You know you have n defenders, because the play assigned it that way
    auto enemyRobots = world->getWorld()->getThem();
    const int numberOfDefenders = 2;
    auto defensivePositions = calculateDefensivePositions(numberOfDefenders, world, enemyRobots);

    // TODO: is there really no better way to set data per role?
    // Use this new information to assign the roles using the dealer.
    // TODO: compute the passing position
    const Vector2 passingPosition = Vector2(-2, -2);

    // Calculate receiver info
    if (stpInfos.find("pass_receiver") != stpInfos.end()) {
        std::cout<<field.getLeftmostX()<<std::endl;
        stpInfos["pass_receiver"].setPositionToMoveTo(calculatePositionToPassTo(world, enemyRobots));
    }
    // Calculate Passer info
    if (stpInfos.find("passer") != stpInfos.end()){
        stpInfos["passer"].setPositionToShootAt(passingPosition);
        stpInfos["passer"].setKickChipType(PASS);
    }

    for (int defenderIndex = 0; defenderIndex < numberOfDefenders; defenderIndex++) {
        std::string defenderName = "defender" + std::to_string(defenderIndex + 1);

        if (stpInfos.find(defenderName) != stpInfos.end()) {
            stpInfos[defenderName].setPositionToMoveTo(defensivePositions[defenderIndex]);
        }
    }

}

std::vector<Vector2> Pass::calculateDefensivePositions(int numberOfDefenders, world_new::World* world, std::vector<world_new::view::RobotView> enemyRobots) {
    std::vector<Vector2> positions = {};

    // 3 robots will defend goal
    for (int i = 0; i < numberOfDefenders; i++) {
        if (i < 3) {
            positions.push_back(world->getField()->getOurGoalCenter());
        } else {
            positions.push_back(enemyRobots[i].get()->getPos());
        }
    }

    return positions;
}
Vector2 Pass::calculatePositionToPassTo(world_new::World* world, std::vector<world_new::view::RobotView> enemyRobots) {
    // Construct a pagmo::problem from our example problem.

    auto pso = pagmo::pso_gen(2);
    //pagmo::problem pro{PassProblem{}};
    PassProblem pro{};
    pro.updateInfoForProblem(world);
    pagmo::population::size_type s = 10;
    auto pop = pagmo::population(pro, s);
    auto evolved = pso.evolve(pop);
    std::cout << evolved << std::endl;
    return Vector2(evolved.champion_x()[0], evolved.champion_x()[1]);

}

bool Pass::isValidPlayToStart(world_new::World* world) noexcept { return true; }

bool Pass::shouldRoleSkipEndTactic() { return false; }

}  // namespace rtt::ai::stp::play
