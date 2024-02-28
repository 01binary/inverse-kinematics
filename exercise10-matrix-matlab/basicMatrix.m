% Forward Kinematics with Joint Matrices (Matlab) - basic.blend

% Define Joint Variables
base = 0;
shoulder = -0.501821;
elbow = 0.904666;
wrist = 0.917576;

% Define Joint Frames
Base = makehgtform('zrotate', base);
Shoulder = ...
  makehgtform('translate', [0, 0, 0.670]) * ...
  makehgtform('yrotate', shoulder);
Elbow = ...
  makehgtform('translate', [0.7, 0, 0]) * ...
  makehgtform('yrotate', elbow);
Wrist = ...
  makehgtform('translate', [0.7, 0.05, 0]) * ...
  makehgtform('xrotate', wrist) * ...
  makehgtform('translate', [0.18, 0, 0]);

% Forward Kinematics
EE = Base * Shoulder * Elbow * Wrist;

% Visualize in Blender
bpy.data.objects["EE"].matrix_world = Matrix([
  [],
  [],
  [],
  [0,0,0,1]
])
