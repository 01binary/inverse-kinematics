% Analytical Inverse Kinematics with Symbols in 4-DOF (Matlab)

% Define Joint Variables as Symbols
% If you don't declare them as "real", you may get imaginary numbers
syms theta1 theta2 theta3 theta4 real

% Define Goal
% Set from FK or Copy from bpy.data.objects["tool"].matrix_world in Blender
EE = ...
  zrotate(-0.528585) * ... % base
  translate(0, 0, 0.670) * ...
  yrotate(-0.585409) * ... % shoulder
  translate(0.7, 0, 0) * ...
  yrotate(1.23597) * ...   % elbow
  translate(0.7, 0.05, 0) * ...
  xrotate(0.917576) * ...  % wrist
  translate(0.18, 0, 0)

% Define Joint Frames
Base = zrotate(theta1);
Shoulder = ...
  translate(0, 0, 0.670) * ...
  yrotate(theta2);
Elbow = ...
  translate(0.7, 0, 0) * ...
  yrotate(theta3);
Wrist = translate(0.7, 0.05, 0) * ...
  xrotate(theta4);
Tool = translate(0.18, 0, 0);

% Define IK Equation
IK = Base * Shoulder * Elbow * Wrist * Tool == EE;

% Decouple Tool Frame
IK = Base * Shoulder * Elbow * Wrist == EE * inv(Tool);

% Decouple Wrist Frame
IK = Base * Shoulder * Elbow == EE * inv(Tool) * inv(Wrist);

% Decouple Elbow Frame
IK = Base * Shoulder == EE * inv(Tool) * inv(Wrist) * inv(Elbow);

% System of Nonlinear Equations from IK Equation
LHS = lhs(IK);
RHS = rhs(IK);

E1 = simplify(LHS(1,1) == RHS(1,1));
E2 = simplify(LHS(1,2) == RHS(1,2));
E3 = simplify(LHS(1,3) == RHS(1,3));
E4 = simplify(LHS(1,4) == RHS(1,4));

E5 = simplify(LHS(2,1) == RHS(2,1));
E6 = simplify(LHS(2,2) == RHS(2,2));
E7 = simplify(LHS(2,3) == RHS(2,3));
E8 = simplify(LHS(2,4) == RHS(2,4));

E9 = simplify(LHS(3,1) == RHS(3,1));
E10 = simplify(LHS(3,2) == RHS(3,2));
E11 = simplify(LHS(3,3) == RHS(3,3));
E12 = simplify(LHS(3,4) == RHS(3,4));

% E9 has theta2 and theta3
