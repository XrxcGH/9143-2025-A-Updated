Leviathan (9143 Robot A) articulated model for AdvantageScope.

Component mapping - MUST match the order the robot code publishes poses in
NT://Draggables/Components3d and RobotState/ComponentPoses (see
Constants.LoggingConstants and Dashboard.update()):

  Model    - drive base (follows the robot pose)
  Model_0  - elevator middle stage (rises at half carriage speed)
  Model_1  - elevator carriage
  Model_2  - CorAl arm (pitches about the pivot)

(This robot has no climb mechanism - an earlier version of this file listed
Model_1 as "climb", which was a template leftover.)

VERIFY on first AdvantageScope session: if the exported glTF components are
in a different order, either re-export in this order or remap in config.json.
