% Analytical Inverse Kinematics with Symbols

% Define Joint Variables
base = 0.4;
shoulder = 0.8;
elbow = -0.6;

% Define Joint Frames
Base = ...
  dh(a = 0,       d = 0.11518, alpha = pi / 2,  theta = theta1) * ...
  dh(a = -0.013,  d = 0,       alpha = 0,       theta = 0);
Shoulder = ...
  dh(a = 0.4173,  d = 0,       alpha = 0,       theta = theta2);
Elbow = ...
  dh(a = 0.48059, d = 0,       alpha = 0,       theta = theta3) * ...
  dh(a = 0.008,   d = 0,       alpha = 0,       theta = pi / 4) * ...
  dh(a = 0.305,   d = -0.023,  alpha = 0,       theta = -pi / 4 + 0.959931);

EE = Base * Shoulder * Elbow

% Visualize in Blender
bpy.data.objects["EE"].matrix_world = Matrix([
  [],
  [],
  [],
  [0,0,0,1]
])
