#pragma once

#include <map>
#include <string>
#include "ros/package.h"

namespace rtt {
namespace df {

#define DF_CONFIG_PARAM_FILE ros::package::getPath("roboteam_world") + "/DFParams.json"

class DangerFinderConfig {
public:
	struct SubConfig {
		std::string name;
		std::map<std::string, double> doubles;
		std::map<std::string, int> ints;
		std::map<std::string, bool> bools;
		std::map<std::string, std::string> strings;
	};

	DangerFinderConfig();
	SubConfig getConfigFor(std::string moduleName) const;

private:
	std::map<std::string, SubConfig> configs;
};

}
}
