% Analytical Inverse Kinematics with Symbols in 4-DOF (Matlab)

% Define Joint Variables as Symbols
% If you don't declare them as "real", you may get imaginary numbers
syms base shoulder elbow real

% Define Goal
% Set from FK or Copy from bpy.data.objects["tool"].matrix_world in Blender
EE = ...
  zrotate(-0.528585) * ... % base
  translate(0, 0, 0.670) * ...
  yrotate(-0.585409) * ... % shoulder
  translate(0.7, 0, 0) * ...
  yrotate(1.23597) * ...   % elbow
  translate(0.7, 0.05, 0) * ...
  translate(0.18, 0, 0);

% Define Joint Frames
Base = zrotate(base);
Shoulder = ...
  translate(0, 0, 0.670) * ...
  yrotate(shoulder);
Elbow = ...
  translate(0.7, 0, 0) * ...
  yrotate(elbow) * ...
  translate(0.7, 0.05, 0) * ...
  translate(0.18, 0, 0);

% Define IK Equation
IK = Base * Shoulder * Elbow == EE;

% Decouple Elbow Frame
IK = Base * Shoulder == EE * inv(Elbow);

% Solve for Joint Variables
[base, shoulder, elbow] = solve(IK, [base, shoulder, elbow])

% Print Results
vpa(base)
vpa(shoulder)
vpa(elbow)