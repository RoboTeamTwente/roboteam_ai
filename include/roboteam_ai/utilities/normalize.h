#pragma once

#include <proto/RobotCommand.pb.h>
#include <proto/World.pb.h>
#include <proto/WorldBall.pb.h>
#include <proto/WorldRobot.pb.h>
#include <proto/messages_robocup_ssl_geometry.pb.h>
#include <proto/messages_robocup_ssl_referee.pb.h>
#include <roboteam_utils/Mathematics.h>

namespace roboteam_utils {

/**
 * @brief Multiplies all the members of a Ball by -1
 *
 * Pointer is asserted in debug mode
 *
 * @param ball Ball to mutate
 */
void rotate(proto::WorldBall *ball);

/**
 * @brief Multiplies all the members of a Robot by -1
 *
 * Robot pointer is asserted for debug mode
 *
 * @param robot Pointer to the Robot to mutate
 */
void rotate(proto::WorldRobot *robot);

/**
 * @brief Rotates all robots and the ball in the world
 *
 * World* is asserted in debug mode
 *
 * @param world World to mutate
 */
void rotate(proto::World *world);

/**
 * @brief Rotates the referee
 *
 * Pointer is asserted in debug mode
 *
 * @param refereeData Referee to rotate
 */
void rotate(proto::SSL_Referee *refereeData);

/**
 * @brief Rotates all members of a Circular arc
 *
 * Pointer is asserted in debug mode
 *
 * @param arc Arc to rotate
 */
void rotate(proto::SSL_FieldCircularArc *arc);

/**
 * @brief Rotates all members of a line
 *
 * Pointer is asserted in debug mode
 *
 * @param line Line to rotate
 */
void rotate(proto::SSL_FieldLineSegment *line);

/**
 * @brief Rotate all field lines and field arcs in a field
 *
 * Pointer is asserted in debug mode
 *
 * @param field Field to change
 */
void rotate(proto::SSL_GeometryFieldSize *field);

/**
 * @brief Rotates the x, y and w of a command
 *
 * Cleans the rotated w
 *
 * Pointer is asserted in debug mode
 *
 * @param command Command to rotate
 */
void rotate(proto::RobotCommand *command);

}  // namespace roboteam_utils
