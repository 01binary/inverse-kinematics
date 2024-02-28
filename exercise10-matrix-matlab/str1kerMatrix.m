% Forward Kinematics with Joint Matrices (Octave) - str1ker.blend

% Define Joint Variables
base = 0.4;
shoulder = 0.8;
elbow = -0.6;

% Define Joint Frames
Base = ...
  makehgtform('translate', [0, 0, 0]) * ...
  makehgtform('zrotate', base) * ...
  makehgtform('xrotate', pi / 2);
Shoulder = ...
  makehgtform('translate', [-0.013, 0.11518, 0]) * ...
  makehgtform('zrotate', shoulder);
Elbow = ...
  makehgtform('translate', [0.4173, 0, 0]) * ...
  makehgtform('zrotate', elbow) * ...
% Fixed Joint
  makehgtform('translate', [0.48059, 0, -0.023]) * ...
  makehgtform('zrotate', 0.959931) * ...
% Tool
  makehgtform('translate', [0.305, 0, 0]);

% Forward Kinematics
EE = Base * Shoulder * Elbow

% Visualize in Blender
bpy.data.objects["EE"].matrix_world = Matrix([
  [],
  [],
  [],
  [0,0,0,1]
])
