#include "roboteam_world/danger_finder/DangerModule.h"

namespace rtt {
namespace df {

PartialResult::PartialResult() : score(0), flags(0) {}
PartialResult::PartialResult(double score, DangerFlag flags) : score(score), flags(flags) {}

PartialResult& PartialResult::operator+=(const PartialResult& b) {
	score += b.score;
	flags |= b.flags;
	return *this;
}

PartialResult operator+(PartialResult a, PartialResult b) {
	return { a.score + b.score, (DangerFlag)(a.flags | b.flags) };
}

DangerModule::DangerModule(std::string name) : name(name) {}

std::string DangerModule::getName() const { return name; }

DangerFinderConfig DangerModule::cfg() {
	static DangerFinderConfig d;
	return d;
}

boost::optional<DangerModule*> DangerModule::buildModule(std::string name) {
	if (moduleRepo().find(name) != moduleRepo().end()) {
		return moduleRepo().at(name)();
	}
	return boost::none;
}

DangerFinderConfig::SubConfig DangerModule::myConfig() {
	return cfg().getConfigFor(name);
}

std::map<std::string, DangerModule*(*)()>& moduleRepo() {
	static std::map<std::string, DangerModule*(*)()> repo;
	return repo;
}

}
}
