# Ball #

## General ##
The structure that represents the ball, it really just
serves the purpose of representing the ball. Things like position and velocity are represented in it.

## Members ##
`position` -> Instance of `Vector2` that represents the current position.

`velocity` -> Instance of `Vector2` that represents the current velocity.

`visible` -> Boolean that represents whether the ball is currently visible by the camera, if the ball is next to a robot and the camera is looking at it through an angle, then it can't see the ball and `visible` will be `false`.

`expectedEndPosition` -> End position that we _think_ the ball will lay at when it comes to a stop.

`filteredVelocity` -> Velocity but adjusted to match up with physics n stuff, it's used to determine a more realistic end position.

## Functions ## 

`initializeCalculations` -> Calls the 4 calculation methods.

`initBallAtRobotPosition` -> If there is no position for the ball we look at the robot ball sensors for this.

`filterBallVelocity` -> Sets `filteredVelocity`

`updateExpectedBallEndPosition` -> Sets `expectedEndPosition`.

`updateBallAtRobotPosition` -> Updates `expectedEndPosition` and draws to the interface.

`getPosBall` -> Returns `position`

`getVelocityBall` -> Returns `velocity`

`isVisibleBall` -> Returns `visible`

`getExpectedEndPositionBall` -> Returns `expectedEndPosition` 

`getFilteredVelocity` -> Returns `filteredVelocity`

`Ball(proto::WorldBall, World*)` -> Constructs a ball from a proto message + the world data.

## Advice ##
There's a few black boxes in here that I didn't fix when I refactored the world, cause honestly I didn't feel like it.

So my advice is simple, refactor the 4 calculation functions, `initBallAtRobotPositions`, `filterBallVelocity`, `updateExpectedBallEndPosition` and `updateBallAtRobotPosition`. The last one is most important here because for whatever reason it updates the UI.