#pragma once

#include <map>
#include <string>
#include <list>
#include <type_traits>
#include <array>
#include <boost/variant.hpp>
#include "roboteam_world/robot.h"
#include "roboteam_world/ball.h"
#include "roboteam_utils/Position.h"
#include "roboteam_utils/constants.h"
#include "roboteam_utils/TeamRobot.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/Vector3f.h"
#include "ros/ros.h"

namespace rtt {

using Position = Position;
using Vector2 = Vector2;
using World = roboteam_msgs::World;
typedef unsigned int RobotID;    

/**
 * The result value of a Tracker.
 */    
typedef boost::variant<long, double, std::string, bool, Position> TrackedValue;

/**
 * The possible result types of a Tracker. Used to identify which field of a TrackedValue is set.
 */
enum class TrackedValueType { LONG, DOUBLE, STRING, BOOL, VEC3 };     
    
    
/**
 * @class TrackerResult
 * @author Dennis
 * @date 09/01/17
 * @file opponent_tracker.h
 * @brief The result of a tracking operation. Contains a success flag, result type, and result value.
 * If the value of success is false, the other two fields should not be used.
 */
typedef struct TrackerResult {
    bool success;
    TrackedValue value;
    TrackedValueType type;
    
    TrackerResult() : success(false), value{false}, type(TrackedValueType::BOOL) {}
    TrackerResult(const TrackerResult& tr) : success(tr.success), value{false}, type(tr.type) {
        switch (type) {
            case TrackedValueType::LONG: value = boost::get<long>(tr.value); break;
            case TrackedValueType::DOUBLE: value = boost::get<double>(tr.value); break;
            case TrackedValueType::STRING: value = boost::get<std::string>(tr.value); break;
            case TrackedValueType::BOOL: value = boost::get<bool>(tr.value); break;
            case TrackedValueType::VEC3: value = boost::get<Position>(tr.value); break;
        }
    }
    ~TrackerResult(){}
} TrackerResult; 
    
/**
 * @class TrackerModule
 * @author Dennis
 * @date 09/01/17
 * @file opponent_tracker.h
 * @brief Abstract base class for trackers.
 */
class TrackerModule {
public:
    virtual ~TrackerModule() {}
    
    /**
     * @brief Updates this tracker with a new world state.
     * @param world The current world state.
     */
    virtual void update(const World& world) = 0;
    
    /**
     * @brief Get a unique name for this module.
     * @return  The name.
     */
    virtual const std::string name() const = 0;
    
    /**
     * @brief Retrieve the most current tracking result for a certain robot, if possible.
     * @param id The robot to track.
     * @return A TrackerResult with the most recent data, or one which has the success flag set to false
     * if it is not (currently) possible to calculate a result for this robot.
     */
    virtual TrackerResult calculate_for(const TeamRobot& bot) const = 0;
    
    bool operator==(const TrackerModule& other) const;
    
    /**
     * @brief Checks whether this module wants to be updated right now.
     * The intent is that modules performing expensive operations may not
     * want to update every tick in order to save CPU time.
     * @return Whether or not this module should be updated now.
     */
    virtual bool want_update();
    
};    
    
/**
 * @class OpponentTracker
 * @author Dennis
 * @date 09/01/17
 * @file opponent_tracker.h
 * @brief Main tracker class. Aggregates TrackerModules and updates them.
 */
class Tracker {
    
public:
    ~Tracker();
    
    /**
     * @brief Tick all registered TrackerModules with the current world state.
     * @param world The current world state.
     */
    void update(const World& world);
    
    /**
     * @brief Add a new TrackerModule if one with the same name does not already exist.
     * Managing the module's lifecycle becomes the OpponentTracker's responsibility - the
     * caller should not delete it.
     * @param module The TrackerModule to add.
     * @return Whether or not the module was actually added.
     */
    bool add_module(TrackerModule* module);
    
    /**
     * @brief Checks whether a module is registered in this OpponentTracker.
     * @param module The module to check.
     * @return True if the module is registered.
     */
    bool has_module(const TrackerModule& module) const;
    
    /**
     * @brief Checks whether a module with a given name is registered in this OpponentTracker.
     * @param name The name to check.
     * @return True if a registered module has the given name.
     */
    bool has_module(const std::string& name) const;
    
    /**
     * @brief Unregisters a module. If successful, that module will be deleted.
     * @param module The module to remove.
     * @return True if the module was registered and has been deleted. False if 
     * the module was not registered before.
     */
    bool remove_module(const TrackerModule& module);
    
    /**
     * @brief Unregisters a module by name. If successful, that module will be deleted.
     * @param name The name of the module to remove.
     * @return True if a module with the given name was registers and has now been deleted.
     * False if no module with that name was found.
     */
    bool remove_module(const std::string& name);
    
    /**
     * @brief Type-safe way of retrieving a registered module by name.
     * @param name The name of the module to get.
     * @return A const pointer to the module if one with the right name is registered, nullptr otherwise.
     */
    template <class T>
    typename std::enable_if<std::is_base_of<TrackerModule, T>::value, const T*>::type get_module_safe(const std::string& name) const {
        for (const TrackerModule* module : modules) {
            if (name == module->name()) {
                return static_cast<const T*>(module);
            }
        }
        return nullptr;
    }
    
    /**
     * @brief Gets a registered module by name.
     * @param name The name of the module to get.
     * @return A const pointer to the module if one with the right name is registered, nullptr otherwise.
     */
    const TrackerModule* get_module(const std::string& name) const;
    
    
private:
    std::list<TrackerModule*> modules;
};

}