# roboteam_ai
This repository will replace 'roboteam_tactics'

Currently WIP


### dependencies
Coverage:
``` sudo apt-get install gcovr lcov ```

### doxygen support
To generate documentation:
```
$ sudo apt install doxygen
$ doxygen doxygen
```

### Contributing to roboteam ai
#### How does it work?
`roboteam_ai` receives input from 3 sources:
- Referee which gives any relevant commands of the Referee to the game
- World State, which contains robot and ball positions and vectors and dynamic information (speed, acceleration)
- GeometryData, which contains static information on the world (field lines, goal position, etc.)

'roboteam_ai' then creates strategies and tactics out of these using behavior trees. If you want to know more about these, see (insert reference)

At the lowest level, these are converted into robotcommands. Robotcommands are again send over a ROS topic to `roboteam_robothub`, which are then sent to the robots. At this point the robotcommands are also verified to be in the right output format; you can find which units and ranges you can work in here.

