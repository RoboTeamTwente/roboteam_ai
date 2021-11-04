# Path Planning Module
The purpose of this module is to compute a path for each robot, that manages to avoid obstacles (other robots, the defense area, staying inside the field, etc.) and, using velocity control, have the robot reach its destination as fast as possible. The structure is divided into multiple parts:
* Path Planning;
* Path Tracking;
* Collision Detection;
* The integration of all of the above.

## Path Planning
The purpose of the Path Planning algorithm is to generate a path / paths that the current robot has to follow, in order to reach its destination without colliding with obstacles. The path is implemented as a tree, containing the position of the node (_consider adding more information - like the time needed to get there_) and the node's parent (the root will have a null parent). By calling the collision detector, a new point can be evaluated in terms of collisions, to which the distance to the goal, cost to get there, etc. can be added to determine whether it should be added to the path or not.

### Current Implementation
1. Check if the current path point is valid for the path (whether there is a collision between the current point and its parent)
2. If there are collisions, determine the collision point and branch to the sides of the obstacle. Add the 2 new candidates to a path queue and go back to step 1
3. If there are no collisions, update the current best path (in case the current point is closer to the destination than the previous best)
4. From the current position, we trace a path directly to the target position and repeat step 2
5. If there are no collisions to reach the target, return the path
6. The algorithm ends when:
* A path to the target is found
* The algorithm reaches a maximum number of iterations or branches too much (returning the current best path)
* The current point is close (e.g. within 10cm) of the target (returning the current path)

The algorithm currently ignores the points that are outside of the field. The result is a sparse path that avoids obstacles according to some avoidance distance (the branching distance from the collision point).

## Path Tracking
After obtaining the path, a velocity and angle has to be computed (as the robots use velocity control). Depending on the planning algorithm, the path may or may not be smooth, so the robot will experience bottlenecks on the corners. The purpose of this module is following a given path as efficiently as possible.

### Current Implementation
1. Remove the first point in the path if it has been reached (distance < some threshold)
2. The velocity is computed by filtering the difference between the current position and the target position through a decoupled PID controller.
3. The parameters of the filter are taken directly from the interface, depending on the PID type needed (e.g. the keeper will have different PID values than default).

## Collision Detector
Contains helper functions to determine:
- robot collisions (and their position - currently returns only the collision with the first robot ID, consider taking the closest collision)
- collision with the defense area (and its position)
- whether a point is in field or not

These helper functions are just a wrapper for the field computation functions.

## Integration
The main module just creates all the above modules and is responsible for their life cycle + calling them in the right order. Also, it incorporates some extra checks:
1. If the target is outside of the field, no path is computed.
2. If the robot is close to the target, but can't reach it due to collisions, it will not ram the obstacle (no further path)
3. A path is recomputed if either one of these conditions is true:
* there is no path
* the target changed
* a collision is detected between the moving robot and its next path way point

## Improvements
Consider experimenting with / researching different algorithms for different tasks. Consider also using different algorithms for each robot (and think before whether it would be better to have separate algorithms for each robot or not). Ideas:
1. Path Planning
- Voronoi (slow for many robots, also large avoidance - an implementation already exists in the code base)
- Split the search area into grids and check grid occupancy
- RRT* augmented with Particle Swarm (or other RRT implementations - many teams use RRT for their path planning)
- Magnetic field lines (basically smart picking for RRT - draw magnetic field lines from the current position to the target and sample points from these lines to try to build a path)
2. Path Tracking
- Pure Pursuit (use a look ahead distance to try to track a path)
- Fast Marching (personally I didn't understand this, but there are papers on it)
- Bezier (interpolate the path points and take the path derivative(?) to track it)
- Potential Field (each obstacle gives a repelling force proportional to distance, while the goal has an attracting force - maybe combine with planning)
- Couple the PID controllers together - also taking into account the robots maximum velocity
- Bang bang control (Simplify the control into Accelerate/Decellerate at the highest rate and coasting at the maximal velocity and choose one of these 3  )
3. Others
- Maybe the velocity can be computed directly in the path planning, according to different metrics, and then the tracking can just filter the velocity or something
- Ignoring collisions for far away points (e.g. after 3seconds of path or 2m away)
- Extend the current implementation by taking into account the time needed to reach the points (or use other metrics to compare paths)
- Rewrite the collision detector to consider custom areas to avoid (specified either by flags or passing an array of shapes to it)
