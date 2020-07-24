# STP
STP stands for Skill Tactic Play. It's nice. The goal ( :D ) is to divide robot behaviour into small, reusable, testable blocks that can be used to create more complex behaviour, as well as create a maintainable system design where each layer has a single clear responsibility. Keep in mind that this wiki is a WIP, the structure is still being adjusted whenever it is discovered that something does not fit.

## Skill
A skill is an atomic class. It encapsulates a "basic" robot behaviour, like kick or go to position. Skills return Status::Success or Status::Running. They never fail. They use the structure STPInfo to receive the data the skill needs to execute, like what dribbler speed it should run at. It does not calculate anything by itself.

## Tactic
A tactic is a state machine of skills. It is to be kept quite small to make it overseeable. It moves linearly through the skills, progressing when the current skill is succesful. The tactic has a calculateInfoForSkill function, making it responsible for ensuring the skill has the right information. For example, in the BlockRobot tactic, in this function the tactic calculates exactly what position the robot should move to in order to block the desired robot. Which robot is desired is decided by the Play.

### End-Tactics
A tactic can also be an end-tactic. In this case, the tactic will return Status::Running forever, because it represents robot behaviour that has no clear endpoint, such as blocking a robot. However, the role can still progress beyond the end-tactic. This is decided by the play. To illustrate:

**Defender: BlockRobot, Intercept**

Blocking the robot can be done forever, hence why BlockRobot is an end-tactic. However, at some point the ball will be shot. At that point, the play can realise this and make one of the robots intercept the ball. The play will look through all the defenders and progress one of them to the Intercept tactic.

### Resetting a Tactic
It can happen that the tactic is already in the next state of the state machine, but it needs to execute the first part again. For example:

**Receive: GoToPos, Rotate, SetDribbler**

Suppose the tactic has gone to the right position and is now rotating. However, the play decides the receiver should actually move a little bit to the right, but the receiver is already in the next skill. This is solved by checking, if the tactic is not in the first state of execution, whether it needs to reset. 
Going back to the Receive skill, the reset condition would be if robot.pos != target_point. This is because then it has already finished the GoToPos, so it needs to reset and do that again. 

### Failing a Tactic
Tactics are small and modular. That means they rely on each other a lot. For example, these tactics used in the Pass role:

**Pass: GetBall, ShootAtPos**

ShootAtPos assumes the robot already has the ball. Therefore, it should fail if the robot does not have the ball anymore, and the state machine should reset. In this case, that means the state machine will try and get the ball again. 

## Role
A role is a state machine of tactics. It progresses whenever a tactic is succesful, unless the tactic is an end-tactic. In that case, as explained before, the play will progress the role past the end-tactic if it is deemed necessary.

## Play
The play is where the decision making happens. This is done in a central location to promote better cooperation between the robots. The play is responsible for making sure the roles are divided to the robots. If there is a pass, the play decides who passes to who. 

Furthermore, the play is responsible for any decision-making related to the tactics. For example, the BlockRobot tactic should receive a BlockDistance enum from the play that decides how close our robot should be to their robot. Another example is which enemy robots should be blocked by the defenders.

Play is meant to be used in a modular way. Small plays with small roles is the intended usage of this part of the structure. This means plays stay small and overseeable. For example, a Pass play, an Attack play, etc. Then the playdecider will lead the progression through the small plays and decide every time a play is finished which play would be the best candidate to start with. This is expected to lead to dynamic behaviour while keeping the structure small.