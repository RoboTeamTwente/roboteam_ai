# Robot #

## General ##
The structure that represents a robot, it represents the team it's on, and some constants. POD.

## Team ##
The enum Team in [team.hpp](https://github.com/RoboTeamTwente/roboteam_ai/blob/development/include/roboteam_ai/world_new/Team.hpp) is fairly simple. A robot is either `us`, meaning it belongs to us, `them`, meaning it belongs to them or `both`, which is invalid. 

## Members ##
`id` -> The ID of the robot (between 1 and 11, 0 is invalid iirc)

`team` -> Enum value on which team it is, read the [section above](#team)

`pos` -> Current position of the robot

`vel` -> Current velocity of the robot

`angle` -> Current angle of the robot

`pidPreviousVel` -> The previous pid velocity.

`distanceToBall` -> Distance from the robot to the ball

`angleDiffToBall` -> Angle offset between the kicker and the ball.

`angularVelocity` -> Angular velocity of the robot.

`batteryLow` -> Flag that indicates whether the battery of the robot is low.

`dribblerState` -> State of the dribbler, essentially the number indicates how fast it goes.

`previousDribblerState` -> State of dribbler in the previous tick, dito.

`timeDribblerChanged` -> The time at which the state of the dribbler changed.

`timeToChangeOneDribblerLevel` -> The time which it takes to change from for example 0 to 1 for the dribbler.

`workingDribbler` -> True if dribbler works, false if not.

`workingBallSensor` -> Dito but for sensor.

`seesBall` -> True if the ball sensor picks up the ball, false if not.

`ballPos` -> Position of the ball (if seesBall == true)


## Functions ## 

### Trivial getters and setters, therefore will **not** be covered ##

## Advice ##
Honestly it's POD I don't have any advice for this one.
