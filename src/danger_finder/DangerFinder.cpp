#include "roboteam_world/danger_finder/DangerFinder.h"
#include "ros/ros.h"
#include "roboteam_world/danger_finder/DistanceModule.h"
#include "roboteam_world/danger_finder/OrientationModule.h"
#include <boost/optional.hpp>

namespace rtt {

namespace df {

WorldBase* DangerFinder::world = nullptr;

boost::optional<roboteam_msgs::WorldRobot> getWorldBot(int id, bool ourTeam) {
	auto world = LastWorld::get();
	std::vector<roboteam_msgs::WorldRobot> vec = ourTeam ? world.us : world.them;
	for (const auto& bot : vec) {
		if (bot.id == id) return bot;
	}
	return boost::none;
}

const std::vector<df::DangerModule*> DangerFinder::modules() {
	const static std::vector<df::DangerModule*> vec = {
			new df::DistanceModule(),
			new df::OrientationModule()
		};
	return vec;
};

DangerFinder::DangerFinder() : stopping(false), running(false), ranOnce(false) {}

DangerFinder& DangerFinder::instance() {
	static DangerFinder local_df;
	return local_df;
}

void DangerFinder::ensureRunning(int itsPerSecond) {
	if (!instance().running) {
		instance().start(itsPerSecond);
	}
}

void DangerFinder::start(int iterationsPerSecond) {
	unsigned delay = (unsigned) (1000 / iterationsPerSecond);
	runner = std::thread(&DangerFinder::loop, this, delay);
	runner.detach();
	running = true;
}

void DangerFinder::loop(unsigned delayMillis) {
	std::chrono::milliseconds delay(delayMillis);

	while (!stopping) {
		calculate();
		std::this_thread::sleep_for(delay);
	}
}

void DangerFinder::calculate() {
	DangerData data;
	const auto worldMsg = world->as_message();
	for (const auto& bot : worldMsg.them) {
		DEBUGLN("Calculating danger for bot %d", bot.id);
		df::PartialResult pr;
		for (auto& module : modules()) {
			auto t = module->calculate(bot);
			DEBUGLN("Module %s: Score=%f Flags=0x%02X", module->getName().c_str(), t.score, t.flags);
			pr += t;
		}
		DEBUGLN("Results: Score=%f, Flags=0x%02X\n", pr.score, pr.flags);
		data.flags[bot.id] = pr.flags;
		data.scores[bot.id] = pr.score;
		data.dangerList.push_back(bot.id);
	}
	std::sort(data.dangerList.begin(), data.dangerList.end(), [data](const int& a, const int& b) {
		return data.scores.at(a) > data.scores.at(b);
	});
	DEBUG("Danger list: [ ");
	for (int i : data.dangerList) {
		DEBUG("%d ", i);
	}
	DEBUGLN("]\n");

	while (!mutex.try_lock()) {}
	try {
		mostRecentData = data;
	} catch (...) {
		mutex.unlock();
		throw;
	}
	mutex.unlock();
	ranOnce = true;
}

bool DangerFinder::isRunning() const {
	return running;
}

void DangerFinder::stop() {
	stopping = true;
	runner.join();
	running = false;
}

DangerData DangerFinder::getMostRecentData() {
	ensureRunning();
	if (!ranOnce) {
		calculate();
	}
	DangerData t;
	while (!mutex.try_lock()) {}
	try {
		t = mostRecentData;
	} catch (...) {
		mutex.unlock();
		throw;
	}
	mutex.unlock();
	return t;
}

DangerData DangerFinder::calculateDataNow() {
	calculate();
	return getMostRecentData();
}

}

}
