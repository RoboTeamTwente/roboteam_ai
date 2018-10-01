#include "DangerFinderConfig.h"
#include "roboteam_utils/json.hpp"
#include "ros/ros.h"
#include <fstream>

namespace rtt {
namespace ai {
namespace dangerfinder {

DangerFinderConfig::DangerFinderConfig() {
	reload();
}

DangerFinderConfig::SubConfig DangerFinderConfig::getConfigFor(std::string moduleName) const {
	return configs.at(moduleName);
}

bool DangerFinderConfig::reload() {
	nlohmann::json json;
	std::ifstream fileStream(DF_CONFIG_PARAM_FILE);
	ROS_INFO_NAMED("DangerFinder.config", "Reading config file...");
	if (fileStream.fail()) {
		ROS_ERROR_STREAM_NAMED("DangerFinder.config", "Failed to read parameter file:" << DF_CONFIG_PARAM_FILE);
		return false;
	}
	json << fileStream;
	if (!json.is_array()) {
		ROS_ERROR_STREAM_NAMED("DangerFinder.config", "Root element of " << DF_CONFIG_PARAM_FILE << " is not an array!");
		return false;
	}
	for (const nlohmann::json& sub : json) {
		ROS_INFO_STREAM("Reading JSON for module " << sub["module"]);
		if (!sub.is_object()) {
			ROS_ERROR_STREAM_NAMED("DangerFinder.config", "An element of the root array in "
					<< DF_CONFIG_PARAM_FILE << " is not an object!");
			return false;
		}
		SubConfig cfg;
		cfg.name = sub["module"];
		for (nlohmann::json::const_iterator it = sub["params"].cbegin(); it != sub["params"].cend(); ++it) {
			switch ((nlohmann::basic_json<>::value_t) *it) {
			case nlohmann::basic_json<>::value_t::number_float:
				cfg.doubles[it.key()] = it.value();
				break;
			case nlohmann::basic_json<>::value_t::number_integer:
			case nlohmann::basic_json<>::value_t::number_unsigned:
				cfg.ints[it.key()] = it.value();
				break;
			case nlohmann::basic_json<>::value_t::boolean:
				cfg.bools[it.key()] = it.value();
				break;
			case nlohmann::basic_json<>::value_t::string:
				cfg.strings[it.key()] = it.value();
				break;
			default:
				ROS_WARN_STREAM_NAMED("DangerFinder.config", "Unknown parameter type in subconfig " << cfg.name << ": " << it.key());
			}
		}
		configs[cfg.name] = cfg;
		if (sub.value("enabled", false)) {
			activeModules.push_back(cfg.name);
		}
	}
	return true;
}

std::vector<std::string> DangerFinderConfig::getActiveModules() const {
	return activeModules;
}

} // dangerfinder
} // ai
} // rtt
