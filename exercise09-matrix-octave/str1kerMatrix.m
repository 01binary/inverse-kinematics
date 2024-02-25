% Forward Kinematics with Joint Matrices (Octave) - str1ker.blend

% Define Joint Variables
base = 0.4;
shoulder = 0.8;
elbow = -0.6;

% Define Joint Frames
pkg load matgeom
Base = ...
  createTranslation3d(0, 0, 0) * ...
  createRotationOz(base) * ...
  createRotationOx(pi / 2);
Shoulder = ...
  createTranslation3d(-0.013, 0.11518, 0) * ...
  createRotationOz(shoulder);
Elbow = ...
  createTranslation3d(0.4173, 0, 0) * ...
  createRotationOz(elbow) * ...
% Fixed Joint
  createTranslation3d(0.48059, 0, -0.023) * ...
  createRotationOz(0.959931) * ...
% Tool
  createTranslation3d(0.305, 0, 0);

EE = Base * Shoulder * Elbow

% Visualize in Blender
bpy.data.objects["EE"].matrix_world = Matrix([
  [],
  [],
  [],
  [0,0,0,1]
])
