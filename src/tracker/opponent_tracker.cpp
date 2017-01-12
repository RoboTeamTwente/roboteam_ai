#include "roboteam_world/tracker/opponent_tracker.h"
#include <algorithm>

namespace rtt {
    
OpponentTracker::~OpponentTracker() {
    for (auto module : modules) {
        delete module;
    }
}    
    
void OpponentTracker::update(const World& world) {
    for (TrackerModule* module : modules) {
        if (module->want_update()) {
            module->update(world);
        }
    }
}

bool OpponentTracker::add_module(TrackerModule* module) {
    if (!module) return false;
    if (std::find(modules.begin(), modules.end(), module) != modules.end()) {
        return false;
    }
    modules.push_back(module);
    return true;
}

bool OpponentTracker::has_module(const TrackerModule& module) const {
    return has_module(module.name());
}

bool OpponentTracker::has_module(const std::string& name) const {
    for (const TrackerModule* module : modules) {
        if (module->name() == name) {
            return true;
        }
    }
    return false;
}

const TrackerModule* OpponentTracker::get_module(const std::string& name) const {
    for (const TrackerModule* module : modules) {
        if (name == module->name()) {
            return module;
        }
    }
    return nullptr;
}

bool TrackerModule::operator==(const TrackerModule& other) const {
    return name() == other.name();
}

bool TrackerModule::want_update() {
    return true;
}

bool OpponentTracker::remove_module(const TrackerModule& module) {
    return remove_module(module.name());
}

bool OpponentTracker::remove_module(const std::string& name) {
    for (TrackerModule* module : modules) {
        if (module->name() == name) {
            modules.remove(module);
            delete module;
            return true;
        }
    }
    return false;
}

}