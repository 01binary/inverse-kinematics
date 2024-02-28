% Analytical Inverse Kinematics with Symbols in 3-DOF (Matlab)

% Define Joint Variables
% If you don't declare them as "real", you may get imaginary numbers
syms base shoulder elbow real

% Define Goal
% Set from FK or Copy from bpy.data.objects["tool"].matrix_world in Blender
EE = ...
  dh(a = 0,       d = 0.11518, alpha = pi / 2,  theta = 1.5708) * ...
  dh(a = -0.013,  d = 0,       alpha = 0,       theta = 0) * ...
  dh(a = 0.4173,  d = 0,       alpha = 0,       theta = 0.7974) * ...
  dh(a = 0.48059, d = 0,       alpha = 0,       theta = -0.5961) * ...
  dh(a = 0.008,   d = 0,       alpha = 0,       theta = pi / 4) * ...
  dh(a = 0.305,   d = -0.023,  alpha = 0,       theta = -pi / 4 + 0.959931);

% Define Joint Frames
Base = ...
  dh(a = 0,       d = 0.11518, alpha = pi / 2,  theta = base) * ...
  dh(a = -0.013,  d = 0,       alpha = 0,       theta = 0);
Shoulder = ...
  dh(a = 0.4173,  d = 0,       alpha = 0,       theta = shoulder);
Elbow = ...
  dh(a = 0.48059, d = 0,       alpha = 0,       theta = elbow) * ...
  dh(a = 0.008,   d = 0,       alpha = 0,       theta = pi / 4) * ...
  dh(a = 0.305,   d = -0.023,  alpha = 0,       theta = -pi / 4 + 0.959931);

% Define IK Equation
IK = Base * Shoulder * Elbow == EE;

% Decouple Elbow from Base and Shoulder
IK = Base * Shoulder == EE * inv(Elbow);

% Solve for Joint Variables
[base, shoulder, elbow] = solve(IK, [base, shoulder, elbow])

% Print Results
vpa(base)
vpa(shoulder)
vpa(elbow)
