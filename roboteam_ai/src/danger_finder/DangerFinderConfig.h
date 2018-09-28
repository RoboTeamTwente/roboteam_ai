#pragma once

#include <map>
#include <string>
#include "ros/package.h"

namespace rtt {
namespace ai {
namespace dangerfinder {

/**
 * This is the path to the parameter JSON file. DangerFinder will not work if this file
 * cannot be found or parsed.
 */
#define DF_CONFIG_PARAM_FILE ros::package::getPath("roboteam_world") + "/DFParams.json"

/**
 * \class DangerFinderConfig
 * \brief Stores the configuration for DangerFinder and its modules.
 */
class DangerFinderConfig {
public:
	/**
	 * \struct SubConfig
	 * \brief Blackboard-like structure which stores parameters for a single module.
	 */
	struct SubConfig {
		std::string name;
		std::map<std::string, double> doubles;
		std::map<std::string, int> ints;
		std::map<std::string, bool> bools;
		std::map<std::string, std::string> strings;
	};

	/**
	 * \brief Parses the DF_CONFIG_PARAM_FILE.
	 */
	DangerFinderConfig();

	/**
	 * \function getConfigFor
	 * \brief Shortcut to get the SubConfig for a specific module.
	 */
	SubConfig getConfigFor(std::string moduleName) const;

	/**
	 * \function reload
	 * \brief Reloads the configuration from the DF_CONFIG_PARAM_FILE.
	 * \return Whether or not the loading was successful. The configuration should
	 * not be used if it was not successfully loaded.
	 */
	bool reload();

	std::vector<std::string> getActiveModules() const;
private:
	std::map<std::string, SubConfig> configs;
	std::vector<std::string> activeModules;
};

} // dangerfinder
} // ai
} // rtt
