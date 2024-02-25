% Forward Kinematics with Joint Matrices (Matlab) - kuka.blend

% Define Joint Variables
shoulder = -0.53058;
bicep = 0.949459;
elbow = 0.031416;
forearm = 0.631809;
wrist = 0.897099;
palm = 0;

% Define Joint Frames
Shoulder = ...
  makehgtform('translate', [0, 0, 0.203]) * ...
  makehgtform('zrotate', shoulder);
Bicep = ...
  makehgtform('translate', [0.075, 0.0735, 0.13]) * ...
  makehgtform('yrotate', bicep);
Elbow = ...
  makehgtform('translate', [0, -0.0055, 0.27]) * ...
  makehgtform('yrotate', elbow);
Forearm = ...
  makehgtform('translate', [0.106, -0.068001, 0.09]) * ...
  makehgtform('xrotate', forearm);
Wrist = ...
  makehgtform('translate', [0.187, -0.029, 0]) * ...
  makehgtform('yrotate', wrist);
Palm = ...
  makehgtform('translate', [0.052, 0.029, 0]) * ...
  makehgtform('xrotate', palm);
Tool = ...
  makehgtform('translate', [0.03, 0, 0]);

EE = Shoulder * Bicep * Elbow * Forearm * Wrist * Palm * Tool

% Visualize in Blender
bpy.data.objects["EE"].matrix_world = Matrix([
  [],
  [],
  [],
  [0,0,0,1]
])