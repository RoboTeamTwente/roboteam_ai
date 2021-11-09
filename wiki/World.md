# World #

## General ##
[World](https://github.com/RoboTeamTwente/roboteam_ai/blob/development/include/roboteam_ai/world_new/World.hpp#L36) by itself is a very interesting conversation, because we don't just have a single world.
We have two worlds, one called [roboteam_world](http://github.com/roboteamtwente/roboteam_world), and one that we refer 
to as [ai world](https://github.com/RoboTeamTwente/roboteam_ai/blob/development/include/roboteam_ai/world_new/World.hpp)

The world structure simply represents the current world and the history. 
It keeps track of past worlds, and creates new worlds from [protobuf messages](https://developers.google.com/protocol-buffers).

The incoming protobuf messages are pushed back to the history through usage of the [updateWorld](https://github.com/RoboTeamTwente/roboteam_ai/blob/development/include/roboteam_ai/world_new/World.hpp#L108) method.

updateWorld itself does **not** lock the world itself, that's the task of the caller.
In a recent refactor I refactored the world to a way where it could not be used without acquiring a mutex, 
and therefore temporarily acquiring the only write and read access, which is something very important.
Not doing so would result in data races.

The way the caller is forced to lock the world is through a static function called [instance](https://github.com/RoboTeamTwente/roboteam_ai/blob/development/include/roboteam_ai/world_new/World.hpp#L65). 
This function returns a structure of type [AcquireInfo](https://github.com/RoboTeamTwente/roboteam_ai/blob/development/include/roboteam_ai/world_new/World.hpp#L52). 

This structure heavily relies on the principle of [RAII](https://en.cppreference.com/w/cpp/language/raii), which is essentially `clean up after yourself`.
It creates a scoped lock, which locks the mutex inside world, and until the `AcquireInfo` goes out of scope, it shall
stay locked. The way this is done in code is the following.

The `auto const& [x, y] = ...;` syntax below is called [structured binding](https://en.cppreference.com/w/cpp/language/structured_binding). 
It behaves like python tuple unpacking.

```python
def return_tuple() -> (int, int):
    return 1, 2

x, y = return_tuple()
# x == 1
# y == 2
```
```cpp
/**
 * mtx is now your locked mutex, you ignore this.
 * world_ptr is the pointer to the world
 */
auto const& [mtx, world_ptr] = World::instance();
/**
 * Because you ignore `mtx` you just name it `_` usually
 */
auto const& [_, world_ptr] = World::instance();
```

You're not supposed to store this `world_ptr` anywhere unless this mutex is locked.

You cannot call `instance()` again until the current mutex is out of scope. 
Therefore instead of calling it again you usually just pass the `world_ptr` down the call stack.

## Members ##
World holds a variety of members.

`HISTORY_SIZE` -> Indicates the size of the history to be kept inside the world.

`Settings` -> Of type `Settings*`, therefore a non-owning instance of the settings structure. 
It's simply a reference (pointer) to it.

`updateMutex` -> Of type `std::mutex` as aforementioned this is simply the mutex used for acquiring
an instance of the world ot preserve immutability

`updateMap` -> A hashmap of type `std::unordered_map<uint8_t, ProtoFeedback>` which maps the incoming feedback
from the robots to their id, so we can apply this feedback when they get constructed.
We get feedback from the robots directory for things like broken components and batteries.

`history` -> A [circular buffer](https://en.wikipedia.org/wiki/Circular_buffer) used for storing
past worlds, this buffer is appended to when `updateWorld` is called, given that there is a current world.

`currentIndex` -> A 64 bit unsigned integer which keeps track of the current index in `history`.

`currentWorld` -> The current world, as there is no guarantee that we have a current world, this 
is an optional value and therefore wrapped in `std::optional`. This is also the reason that
getting the world data returns an optional view of it.

`currentField` -> the current field, represented in a structure. There will also be a wiki page on the field so
check that one for more information. Like currentWorld there is no guarantee that we have a current representation
of the field, therefore it's wrapped in an `std::optional`.

`lastTick` -> The time at which we got the previous tick from the networking socket. Unix timestamp in seconds.

`tickDuration` -> The difference between `lastTick` and the current incoming tick. Updated on data receive.

`positionControl` -> Robot position controller, used for communication between robots their positions.

## Member functions ##
`World(Settings*)` -> Constructs a world from the current settings, the only available constructor.
Requires the `settings` parameter to not be a nullptr.

`updateFeedback` -> Updates the internal robot feedback hashmap from a new hashmap that's
passed to this function. The hashmap is copied on invocation and then moved into the internal
hashmap.

`updateWorld` -> This is the aforementioned method that updates the world.
```cpp
// pseudo, not complete logic.
if (currentWorld.has_value()) {
    history.emplace_back(currentWorld.value());
} else {
    currentWorld = protoWorld;
}
```

`updateField` -> Updates the internal `currentField` member from a new incoming protobuf Field.

`updateField` -> An overload of the previous updateField, instead it takes an `ai::world::Field` object reference and updates 
the internal world accordingly. Takes ownership of the original data.
Ownership will be explained in a lecture about move semantics.

`updatePositionControl` -> Updates the internal `positionControl` using the new robot positions
in the current world data representation.

`getWorld` -> Gets an optional pointer to `currentWorld`.

`getField` -> Returns `this->currentField`

`getHistoryWorld(index)` -> Returns `this->history[index]`

`getTimeDifference` -> Returns `tickDuration`

`getHistorySize` -> Returns `history.size()`. **Not** guaranteed to be `HISTORY_SIZE`.

`getRobotPositionController` -> Returns a pointer to `positionControl`. 

`reset` -> This function is only available if `RUNNING_TEST` is defined, and completely resets
the world, usage outside of tests can and will result in undefined behavior.

`updateTickTime` -> Updates `lastTick` and `tickDuration`

`setWorld` -> Sets `currentWorld` to the parameter.

`toHistory` -> Pushes a `world` to the history vector.

## Advice for next year ##
The world is really nice right now, it properly abstracts the hard parts away.
It tries to guarantee shared immutability which is a very complicated topic and will be covered
in one of the lectures, specifically about shared ownership and concurrency.

There's an issue where views persist to data that doens't exist anymore reglardless of the current mutex situation.

I'd return copies of WorldData, which could be wrapped like the following:

```cpp
class WorldData {
    // ... stuff
    WorldDataView view() { 
        return this; 
    }
    // ... stuff
};
```

Then you can 


```cpp
std::optional<WorldData> getWorld() { return this->world; }

getWorld().value().view()->...; // stuff
```

I don't know there's a lot of ways you could approach this but returning a copy is one of the few where immutable safety is guaranteed (don't use shared ptr pls).

My advice is simple.
 * Update the history to be an `std::array` instead of `std::vector`. 
 * [Optional] Return copies of data so you don't rely on the user using it correctly.
