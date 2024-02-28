% Forward Kinematics with Joint Matrices (Octave) - basic.blend

% Define Joint Variables
base = 0;
shoulder = -0.501821;
elbow = 0.904666;
wrist = 0.917576;

% Define Joint Frames
pkg load matgeom
Base = createRotationOz(base);
Shoulder = ...
  createTranslation3d(0, 0, 0.670) * ...
  createRotationOy(shoulder);
Elbow = ...
  createTranslation3d(0.7, 0, 0) * ...
  createRotationOy(elbow);
Wrist = ...
  createTranslation3d(0.7, 0.05, 0) * ...
  createRotationOx(wrist) * ...
  createTranslation3d(0.18, 0, 0);

% Forward Kinematics
EE = Base * Shoulder * Elbow * Wrist;

% Visualize in Blender
bpy.data.objects["EE"].matrix_world = Matrix([
  [],
  [],
  [],
  [0,0,0,1]
])
