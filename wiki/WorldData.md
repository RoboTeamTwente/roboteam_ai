# WorldData # 

## General ##
[WorldData]() itself is just a representation of the current world. It receives data from [roboteam_world](http://://github.com/roboteamtwente/roboteam_world) and transforms it into a WorldData structure.

WorldData itself is not that complicated, it serves to be a simple data-only structure with some getters and setters.

## Members ##
WorldData holds a few members, not a whole lot.

`robots` -> A vector of Robot, which represents the robots that are currently in the world.

`robotsNonOwning` -> A vector of robot views to the `robots` vector.

`us` -> A vector of `RobotView`s of the robots that are ours

`them` -> A vector of `RobotView`s of the robots that are on the opposite team.

`ball` -> An optional instance of the ball, can be empty.

`time` -> The timestamp that we received in the protobuf message.

## Member functions ##

`WorldData(World*, protomsg, settings, feedback)` -> Constructs a world data object from the feedback received from robots and the current world + settings + protobuf message `proto::World`.

`setViewVectors` -> Sets `us`, `them` and `robotsNonOwning`. Requires `robots` to not grow anymore.

`getUs` -> Returns `us`

`getThem` -> Returns `them`

`getRobots` -> Returns `robots`

`getBall` -> Returns `ball` wrapped in an optional view.

`weHaveRobots` -> Returns true if the `us` vector is not empty

`getRobotsNonOwning` -> Returns `robotsNonOwning`

`getTime` -> Returns `time`

## Advice ##
Oh boy do I have a lot of advice for this one.

* Rework the `robots` vector to be an `std::array` instead
* Rework the `robotsNonOwning` vector to be an `std::array` instead
* Rework `robotsNonOwning` to be a sorted array of _all_ the robots, not just ours or theirs. Sorted by team.
* Rework `getUs` and `getThem` to return an `std::span` of the robots in the `robotsNonOwning` array.

---
### Optional advice ###
There's some more complicated stuff that I'd like to give as optional advice.

You could be much more memory efficient with the `std::array<Robot, N>` in the future aswel, you could for example abuse the fact that you're allowed to cast to the first member of a type and do something like this.

```cpp
/**
 * An std::span is the following:
 */
namespace std {
  template <typename T>
  struct span {
      T* begin;
      T* end;
  };
}

// Wow look at that proficiency at C++ how 
// am I not hired at GCC yet.

/** 
 * When you create the array
 * (* 2 because 2 teams.)
 */
std::array<Robot, ROBOT_COUNT * 2> robots = {};
for (size_t i = 0; i < our_proto_robots.size(); i++) {
    robots[i] = our_proto_robots[i]; // implicit conversion, maybe needs another argument who knows.
}

for (size_t i = our_proto_robots.size(); i < their_proto_robots.size(); i++) {
    robots[ROBOT_COUNT + i] = their_proto_robots[i]; // implicit conversion, maybe needs another argument who knows.
}

/**
 * So now it's laid out in the following way:
 * { ours, ours, ..., ours, theirs, theirs, ..., theirs };
 * Now returning an std::span<Robot> is trivial.
 */
std::span<Robot> getOurs() {
    return std::span(robots.begin(), robots.begin() + ROBOT_COUNT);
}

std::span<Robot> getTheirs() {
    return std::span(robots.begin() + ROBOT_COUNT, robots.end());
}

/**
 * Now if you want to return a View to a robot that's slightly 
 * more complicated but still possible because the strict aliassing
 * rule allows for casting to the first member ^^
 * RobotView looks like this 
 * class RobotView {
 *     Robot* robot;    
 * };
 * so &*robots.begin() == Robot*, so what we can do is the following.
 * Weellll, not exactly, you'll have to wrap it in some sort of
 * RobotViewArray, or you could of course just have a second std::array<RobotView, N> 
 * to which you return slices.
 */
std::span<RobotView> getUs() {
    return std::span(&*robots_views.begin(), &*robots_views.begin() + ROBOT_COUNT);
}

std::span<RobotView> getThem() {
    return std::span(&*robots_views.begin() + ROBOT_COUNT, &*robots_views.end());
}

std::span<RobotView> getRobotsNonOwning() {
    return std::span(&*robots_views.begin(), &*robots_views.end());
}
```

We think certain members should be pre-initialized on construction of the world. For example, `getRobotClosestToBall`, `whichRobotHasBall`, and things like that are used every tick, and it's computationally faster to set this as a member variable rather than run the calculation each time, depending on how many times they are called, of course. Maybe something like this:
```cpp
WorldData::WorldData(...) {
     setRobotClosestToBall(calculateRobotClosestToBall());
     ...
}
```

