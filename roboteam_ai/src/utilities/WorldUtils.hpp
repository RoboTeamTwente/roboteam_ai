//
// Created by rolf on 18-10-18.
//
//TODO: move all getBot() -like functionality from utils/utils.cpp in tactics here
//TODO: understand when and why TeamRobot is used.
#ifndef ROBOTEAM_AI_WORLDUTILS_HPP
#define ROBOTEAM_AI_WORLDUTILS_HPP
#include "World.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"

#include <boost/optional.hpp>
#include "roboteam_utils/TeamRobot.h"
namespace rtt{
namespace ai{
/**
 * \brief Get all robots active in the world. This is the union of world.us and world.them
 */
    std::vector<roboteam_msgs::WorldRobot> getAllBots(const roboteam_msgs::World& world = World::get_world());

/**
 * \brief Gets all robots active in the world as TeamRobot structures
 */
    std::vector<TeamRobot> getAllTeamBots(const roboteam_msgs::World& world = World::get_world());

/**
 * \brief Gets the robot with the given ID as a WorldRobot
 * @param id The ID of the desired robot
 * @param ourTeam whether or not the robot is in our team
 * @param world The world to search
 * @return An optional containing the robot, or an empty optional if it was not found
 */
    boost::optional<roboteam_msgs::WorldRobot> getWorldBot(unsigned int id, bool ourTeam = true, const roboteam_msgs::World& world = World::get_world());

/**
 * \brief Gets the robot with the given ID as a TeamRobot
 * @param id The ID of the desired robot
 * @param ourTeam whether or not the robot is in our team
 * @param world The world to search
 * @return An optional containing the robot, or an empty optional if it was not found
 */
    boost::optional<TeamRobot> getTeamBot(unsigned int id, bool ourTeam = true, const roboteam_msgs::World& world = World::get_world());
}
}

#endif //ROBOTEAM_AI_WORLDUTILS_HPP
