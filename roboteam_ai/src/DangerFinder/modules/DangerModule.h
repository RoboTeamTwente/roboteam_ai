#ifndef ROBOTEAM_AI_DANGER_MODULE_H
#define ROBOTEAM_AI_DANGER_MODULE_H

#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/World.h"
#include "../PartialResult.h"
#include "../../utilities/World.h"

namespace rtt {
namespace ai {
namespace dangerfinder {

/**
 * \class DangerModule
 * \brief A module to be used in the DangerFinder.
 */
class DangerModule {
public:

    /**
   * \function calculate
   * \brief Performs the calculation for this module for a specific robot.
   * The score should indicate how threatening the robot is, and extra information
   * can be provided through flags.
   */
	virtual PartialResult calculate(const roboteam_msgs::WorldRobot& bot, const roboteam_msgs::World& world = rtt::ai::World::get_world()) = 0;
 protected:
    explicit DangerModule() = default;
	explicit DangerModule(double danger);
	double danger = 0;
};

<<<<<<< HEAD
=======

std::map<std::string, DangerModule*(*)()>& moduleRepo();

template<typename M> class ModuleRegisterer {
 public:
  ModuleRegisterer(std::string name, DangerModule*(*factory)()) {
	  moduleRepo()[name] = factory;
	  std::cout << "[DangerFinder] Registering module" << name << "\n";
  }
};

#define REGISTER_MODULE(name, type)\
	static DangerModule* type ## Factory() { static type* module = new type; return module; }\
	static ModuleRegisterer<type> type ## Registerer(name, &type ## Factory);


>>>>>>> master
} // dangerfinder
} // ai
} // rtt

#endif // ROBOTEAM_AI_DANGER_MODULE_H