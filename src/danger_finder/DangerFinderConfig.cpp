#include "roboteam_world/danger_finder/DangerFinderConfig.h"
#include "roboteam_utils/json.hpp"
#include "ros/ros.h"
#include <fstream>

namespace rtt {
namespace df {

DangerFinderConfig::DangerFinderConfig() {
	nlohmann::json json;
	std::ifstream fileStream(DF_CONFIG_PARAM_FILE);
	ROS_INFO("Reading...");
	if (fileStream.fail()) {
		ROS_ERROR_STREAM("DangerFinderConfig: Failed to read parameter file:" << DF_CONFIG_PARAM_FILE);
		return;
	}

	json << fileStream;

	if (!json.is_array()) {
		ROS_ERROR_STREAM("DangerFinderConfig: Root element of " << DF_CONFIG_PARAM_FILE << " is not an array!");
		return;
	}

	for (const nlohmann::json& sub : json) {
		if (!sub.is_object()) {
			ROS_ERROR_STREAM("DangerFinderConfig: An element of the root array in "
					<< DF_CONFIG_PARAM_FILE << " is not an object!");
			return;
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
				ROS_WARN_STREAM("Unknown parameter type in subconfig " << cfg.name << ": " << it.key());
			}
		}
		configs[cfg.name] = cfg;
	}
}

DangerFinderConfig::SubConfig DangerFinderConfig::getConfigFor(std::string moduleName) const {
	return configs.at(moduleName);
}

}
}
