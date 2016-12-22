#pragma once

#include <map>
#include <string>
#include <list>
#include <type_traits>
#include <array>
#include "roboteam_world/robot.h"
#include "roboteam_world/ball.h"
#include "roboteam_utils/Position.h"
#include "roboteam_utils/constants.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/Vector3f.h"
#include "ros/ros.h"

namespace rtt {

using Position = roboteam_utils::Position;
using Vector2 = roboteam_utils::Vector2;
using World = roboteam_msgs::World;
typedef unsigned int RobotID;    
    
typedef union TrackedValue {
    long long_val;
    double double_val;
    std::string string_val;
    bool bool_val;
    Position pos_val;
    ~TrackedValue() {}
} TrackedValue; 
    
enum class TrackedValueType { LONG, DOUBLE, STRING, BOOL, VEC3 };     
    
typedef struct TrackerResult {
    bool success;
    TrackedValue value;
    TrackedValueType type;
    
    TrackerResult() : success(false), value{false}, type(TrackedValueType::BOOL) {}
    TrackerResult(const TrackerResult& tr) : success(tr.success), value{false}, type(tr.type) {
        switch (type) {
            case TrackedValueType::LONG: value.long_val = tr.value.long_val; break;
            case TrackedValueType::DOUBLE: value.double_val = tr.value.double_val; break;
            case TrackedValueType::STRING: value.string_val = tr.value.string_val; break;
            case TrackedValueType::BOOL: value.bool_val = tr.value.bool_val; break;
            case TrackedValueType::VEC3: value.pos_val = tr.value.pos_val; break;
        }
    }
    ~TrackerResult(){}
} TrackerResult; 
    
class TrackerModule {
public:
    virtual ~TrackerModule() {}
    virtual void update(const World& world) = 0;
    virtual const std::string name() const = 0;
    virtual TrackerResult calculate_for(const RobotID& id) const = 0;
    
    bool operator==(const TrackerModule& other) const {
        return name() == other.name();
    }
};    
    
class OpponentTracker {
    
public:
    ~OpponentTracker();
    void update(const World& world);
    
    bool add_module(TrackerModule* module);
    bool has_module(const TrackerModule& module) const;
    bool has_module(const std::string& name) const;
    bool remove_module(const TrackerModule& module);
    bool remove_module(const std::string& name);
    
    template <class T>
    typename std::enable_if<std::is_base_of<TrackerModule, T>::value, const T*>::type get_module_safe(const std::string& name) const {
        for (const TrackerModule* module : modules) {
            if (name == module->name()) {
                return static_cast<const T*>(module);
            }
        }
        return nullptr;
    }
    
    const TrackerModule* get_module(const std::string& name) const;
    
    
private:
    std::list<TrackerModule*> modules;
};

}