#include "roboteam_world/tracker/tracker.h"
#include <algorithm>

namespace rtt {
    
Tracker::~Tracker() {
    for (auto module : modules) {
        delete module;
    }
}    
    
void Tracker::update(const World& world) {
    for (TrackerModule* module : modules) {
        if (module->want_update()) {
            module->update(world);
        }
    }
}

bool Tracker::add_module(TrackerModule* module) {
    if (!module) return false;
    if (std::find(modules.begin(), modules.end(), module) != modules.end()) {
        return false;
    }
    modules.push_back(module);
    return true;
}

bool Tracker::has_module(const TrackerModule& module) const {
    return has_module(module.name());
}

bool Tracker::has_module(const std::string& name) const {
    for (const TrackerModule* module : modules) {
        if (module->name() == name) {
            return true;
        }
    }
    return false;
}

const TrackerModule* Tracker::get_module(const std::string& name) const {
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


//TODO: they are never used see why not
//
//bool Tracker::remove_module(const TrackerModule& module) {
//    return remove_module(module.name());
//}
//
//bool Tracker::remove_module(const std::string& name) {
//    for (TrackerModule* module : modules) {
//        if (module->name() == name) {
//            modules.remove(module);
//            delete module;
//            return true;
//        }
//    }
//    return false;
//}

}