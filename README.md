# Inverse Kinematics

## 1. Summary

Companion files and source code for [Inverse Kinematics presentation](https://docs.google.com/presentation/d/1boNKH2YIeXIClv_7vQOX7MW-gtbxpkvQ7nWUbjpxxes).

## 2. What is Inverse Kinematics?

+ *Inverse Kinematics*, or *IK*, is a way to calculate robot arm *joint variables* given the desired *end-effector* position and/or orientation (pose).
+ Some types of IK solutions are based on *Forward Kinematics*, or calculating *end effector pose* given *joint variables*.
+ A *software plugin* that can compute *joint variables* given the *robot description* and the desired *goal pose* is called an *IK solver*.

## 3. Purpose

+ Open-source *sampling* solver (KDL) can only find solutions for [6-DOF or greater](https://answers.ros.org/question/152591/kdl-kinematics-solver-limitations/) robots.
+ Open-source *analytical* solver (IKFast) is outdated and has poor support for mimic and fixed joints. The auto-generated C++ code is unreadable.
+ No *tutorials* for inverse kinematics with *less than 6-DOF* (only research papers and course books that require recent exposure to high-level math).
+ Hobbyists working with 3-DOF to 5-DOF robots have no *simple tools* like setup wizards and no accessible instructional materials.

## 4. Background

If you are looking for a more rigorous background, [Introduction to Robotics](https://www.amazon.com/Introduction-Robotics-Analysis-Control-Applications/dp/0470604468/ref) by Saeed Niku is a great resource for inverse kinematics and control systems.

## 5. Table of Contents

- Robot Description
  - Representing Joints
  - Joint Matrix in Octave
  - Joint Matrix in Matlab
  - Denavit-Hartenberg Parameters
  - Denavit-Hartenberg Matrix
- Types of *Inverse Kinematics* solutions
- *Numerical* IK solutions
  - Gradient Descent
  - Newton-Raphson Iterator
- *Sampling-based* IK solutions
- *Geometric* IK solutions

## 6. Tools and Materials

- Blender for visualizing concepts
- Visual Studio Code IDE for debugging C++
- C++ compiler
- Eigen3 library
- Matlab or GNU Octave

## 7. Robot Description

+ Both *forward* and *inverse* kinematics require describing robot *joints*
+ Robot Description defines *links* and *joints* in a *tree structure*
+ *Joint types* include *Revolute* and *Prismatic*
+ *Joints* define *limits*, *offset* from previous joint, and movement *axis*
+ *Links* define *visual* and *simulation* properties
+ Examples: [KUKA KR-5](https://github.com/orsalmon/kuka_manipulator_gazebo), [Str1ker](https://github.com/01binary/str1ker_moveit_config)

## 8. Representing Joints

+ Each joint is represented as a *4x4* (homogenous) *matrix* that *encodes* 3D *offset* and *orientation*.
  + *Revolute* joints: axis and angle of the joint
  + *Prismatic* joints: sliding offset of the joint
+ This matrix will have one or more *joint variables*.
+ All other terms are *constants*.
+ *Multiplying* all joint matrices with their variables filled in will give you the end-effector *pose*.

## 9. Joint Matrix in Octave

To build a 4x4 joint matrix in [GNU Octave](https://octave.org/download):

```
pkg load matgeom
Joint = ...
  createTranslation3d(0, 0, 0) * ...    % constant offset from previous
  createRotationOz(jointVariable) * ... % variable rotation on Z axis
  createRotationOx(pi / 2);             % constant rotation on X axis
```

## 10. Joint Matrix in Matlab

To build a 4x4 joint matrix in [MathWorks Matlab](https://www.mathworks.com/products/matlab.html):

```
Joint = ...
  makehgtform(‘translate’, [0, 0, 0]) * ...   % constant offset from previous
  makehgtform(‘zrotate’, jointVariable) * ... % variable rotation on Z axis
  makehgtform(‘xrotate’, pi / 2)              % constant rotation on X axis
```

## 11. Denavit-Hartenberg Parameters

+ *Denavit-Hartenberg* (DH) *parameters* are a convention for building a joint transformation matrix:
  - Joint offset `d`: translation on Z axis
  - Joint angle `θ`: rotation on Z axis
  - Link length `a`: translation on X axis
  - Joint twist `α`: rotation on X axis
+ Originally devised to save memory & computing power
  - Homogenous transform matrices have `16` redundant parameters, DH has only `4`
+ *Revolute* joints always rotate on *Z axis*
+ *Prismatic* joints always move on *X axis*

## 12. Denavit-Hartenberg Matrix

Denavit-Hartenberg matrix can be built as follows:

```
[cos(θ),  -sin(θ) * cos(α), sin(θ) * sin(α),  a * cos(θ)]
[sin(θ),   cos(θ) * cos(α), -cos(θ) * sin(α), a * sin(θ)]
[0,        sin(α),           cos(α),          d         ]
[0,        0,                0,               1         ]
```

## 13. Types of IK Solutions

+ *Numerical* solvers measure *changes* to end effector *pose* resulting from *changes* in *joint variables*, until they converge on a solution within tolerance.
+ *Sampling-based* solvers build a *graph* of random possible joint states and look for *connections* that lead to end effector reaching *goal pose*.
+ *Analytical* solvers equate the *product of all joint matrices* with the end effector *pose*, and solve the resulting *system of nonlinear equations*.
+ *Geometric* solvers use *trigonometry* to break the problem into multiple steps and solve for each joint variable on its own plane.

## 14. Numerical IK Solvers

+ Iteratively *sample* poses calculated from combinations of *joint variables* by using *Forward Kinematics*.
+ Each joint variable is *increased* or *decreased* based on whether a *change* in this variable resulted in FK pose getting *closer* to or *further* away from the goal.
+ The search *stops* when the pose is within *tolerance of the goal* or upon reaching a *timeout*.
+ Can be done with *Gradient Descent* and *Newton-Raphson Iterator* among others.

## 15. Numerical IK - Gradient Descent

+ The *amount* by which to *increase* or *decrease* each joint variable is calculated using a *gradient equation*:

  `(joints[j][n]-  joints[j][n - 1]) / (fk(joints[j][n]) - fk(joints[j][n-1]))`

+ `j` - joint index (`0` to number of joints)
+ `n` - current iteration
+ `joints` - array of *joint variables*, one for each joint
+ `fk(joints)` - forward kinematics pose given joint variables

## 16 - 17. Numerical IK - Newton-Raphson Iterator

+ The amount by which to *increase* or *decrease* each joint variable is computed by using the *Jacobian matrix* (`J`) *inverse*/*transpose*, which describes *how much* the end effector moves and rotates on *each axis* when each *joint variable* changes.
+ For each joint and end effector axis...

  `J[axis, joint] = (fk(joints… + Δ) - fk(joints…)) / Δ`

  `error = goal - fk(joints)`

  `joints += (Jtrans * error) * damping`

+ Forward kinematics function needs to be able to calculate position of *any link* in the chain.
+ Since Jacobian matrix encodes end effector *movements* in response to a *change* in any of the *joint variables*, *inverting* this matrix will give you the *change* in joint variables resulting from end effector *movement*.
+ This algorithm changes joint variables by *Δ*, determines the resulting *error*, and adjusts the *direction of change* for each joint (for next time) based on error *increasing* or *decreasing*.

## 18. Sampling-Based IK Solvers

+ Create a *graph* of randomly sampled *states* (each state with different *joint variables*).
+ Look for a *path* through this graph that avoids *collisions* and reaches the desired *goal pose*.

## 19. Analytical IK - IKFast

+ [IKFast](https://ros-planning.github.io/moveit_tutorials/doc/ikfast/ikfast_tutorial.html) is a *python script* included in OpenRAVE robotics toolbox.
+ It uses *python equation solving libraries* like [sympy](https://www.sympy.org/en/index.html) and [lapack](https://www.netlib.org/lapack/) to solve an equation that will return end effector pose given joint variables.
+ The *solution* is then converted to *C++* and wrapped into a ROS interface so that it can be used as a *plugin*.

## 20. Analytical IK - Matlab/Octave

+ *Analytical* solvers equate the product of all *joint matrices* with the *end effector pose*.
+ This matrix equation then breaks down into a *system of nonlinear equations* (one for each matrix cell).

```
A1 * A2 * A3 == EE
LHS == EE

LHS[1, 1] == EE[1, 1]
LHS[1, 2] == EE[1, 2]
LHS[1, 3] == EE[1, 3]
LHS[1, 4] == EE[1, 4]

LHS[2, 1] == EE[2, 1]
LHS[2, 2] == EE[2, 2]
LHS[2, 3] == EE[2, 3]
LHS[2, 4] == EE[2, 4]

LHS[3, 1] == EE[3, 1]
LHS[3, 2] == EE[3, 2]
LHS[3, 3] == EE[3, 3]
LHS[3, 4] == EE[3, 4]

% The last row is 0, 0, 0, 1 so we ignore that
```

## 21. Analytical IK - Constant Goal Pose

+ Let's ease into analytical IK by solving for a constant goal pose first.
+ *Setup* the *goal matrix* with constants
+ *Setup* the *joint matrices* with variables and constants
+ *Solve* for *joint variables*:

  `solve(A1 * A2 * A3 == EE, [a1, a2, a3])`

+ This process is *useful for testing* but cannot be used to *build a solver* because in a real application the goal matrix is *unknown until run-time*.

## 22. Analytical IK - Variable Goal Pose

Solving with *variable* goal pose:

+ Setup the *goal matrix* with variables
+ Setup the *joint matrices* with variables
+ Left-hand side (*LHS*): product of all joint matrices
+ Right-hand side (*RHS*): goal pose
+ *De-couple unknowns*: multiply *RHS* by *inverse transform* of the *last* term in *LHS*
+ Setup a *system* of *12 equations* for filled-in cells
+ *Solve* any equations with only *one unknown*, substituting the solution into other equations

## 23. Geometric IK

+ *Solve* for each variable on one *plane* at a time
+ You’ll be using a lot of *inverse trig functions*
+ And *trig identities* (*SOH*-*CAH*-*TOA*)
+ Last but not least, the *Law of Cosines*
