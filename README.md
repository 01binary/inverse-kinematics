# inverse-kinematics

Matlab workspace for calculating inverse kinematic parameters

`dh.m` - Utility function that generates a joint frame matrix given Denavit-Hartenberg parameters

`fk.m` - Forward kinematics function that returns end effector position and orientation given joint angles for this particular robot geometry

`noa.m` - Utility function that solves for unknowns in end effector orientation (normal, orientation approach) given at least three known values. The Danavit-Hartenberg algorithm for inverse kinematics only works if both the orientation and the position of the end effector is known, so this is useful when only the position and one of the angles is known (i.e. angle from base to position)

`ik.m` - Inverse kinematics function that returns joint angles given end effector position and orientation

`demo.blend` - The robot geometry

`demo.m` - The Matlab notebook where forward and inverse kinematic equations for this specific robot are verified
