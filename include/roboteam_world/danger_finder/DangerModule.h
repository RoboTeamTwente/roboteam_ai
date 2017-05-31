#pragma once

#include "roboteam_utils/LastWorld.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/World.h"
#include "DangerFinderConfig.h"

namespace rtt {

typedef unsigned char DangerFlag;

namespace df {

#define DANGER_FINDER_DEBUG false
#define DEBUG(fmt, ...) {\
	if (DANGER_FINDER_DEBUG) ROS_INFO("[DangerFinder] " fmt, ##__VA_ARGS__); \
}
#define DEBUGLN(fmt, ...) DEBUG(fmt "\n", ##__VA_ARGS__)

constexpr DangerFlag DANGER_NONE	  = 0x00; //< Placeholder for empty flags
constexpr DangerFlag DANGER_FREE	  = 0x01; //< The robot could receive the ball easily
constexpr DangerFlag DANGER_CLOSING   = 0x02; //< The robot is closing on our goal
constexpr DangerFlag DANGER_CAN_SHOOT = 0x04; //< The robot has the ball and could should at our goal
constexpr DangerFlag DANGER_CAN_CROSS = 0x08; //< The robot has the ball and could pass it to another opponent near our goal.


/**
 * \struct PartialResult
 * \brief Used internally to accumulate data.
 */
struct PartialResult {
	double score;
	DangerFlag flags;

	PartialResult();
	PartialResult(double score, DangerFlag flags);
	/**
	 * \function operator+=
	 * \brief Sums the scores, ORs the flags.
	 */
	PartialResult& operator+=(const PartialResult& b);
};

/**
 * \function operator+
 * \brief Combines two PartialResults by summing their scores and combining their flags.
 */
PartialResult operator+(PartialResult a, PartialResult b);

/**
 * \class DangerModue
 * \brief A module to be used in the DangerFinder.
 */
class DangerModule {
public:
	/**
	 * \function cfg
	 * \brief Get the static configuration data. The configuration file will be parsed the first
	 * time this function is called.
	 */
	static DangerFinderConfig cfg();
	virtual ~DangerModule() {}

	/**
	 * \function calculate
	 * \brief Performs the calculation for this module for a specific robot.
	 * The score should indicate how threatening the robot is, and extra information
	 * can be provided through flags.
	 */
	virtual PartialResult calculate(const roboteam_msgs::WorldRobot& bot, const roboteam_msgs::World& world = LastWorld::get()) = 0;

	/**
	 * \function getName
	 * \brief Gets the (constant) name of this module, for logging/debugging purposes.
	 */
	std::string getName() const;
protected:
	DangerModule(std::string name);
	DangerFinderConfig::SubConfig myConfig();
private:
	std::string name;
};

}
}
