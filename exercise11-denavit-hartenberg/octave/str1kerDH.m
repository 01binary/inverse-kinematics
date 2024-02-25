% Forward Kinematics with Denavit-Hartenberg parameters
base = 0.4;
shoulder = 0.8;
elbow = -0.6;

Base = ...
  dh(struct("a", 0,       "d", 0.11518, "alpha", pi / 2, "theta", base)) * ...
  dh(struct("a", -0.013,  "d", 0,       "alpha",0,       "theta", 0));
Shoulder = ...
  dh(struct("a", 0.4173,  "d", 0,       "alpha",0,       "theta", shoulder));
Elbow = ...
  dh(struct("a", 0.48059, "d", 0,       "alpha",0,       "theta", elbow)) * ...
  dh(struct("a", 0.008,   "d", 0,       "alpha",0,       "theta", pi / 4)) * ...
  dh(struct("a", 0.305,   "d",-0.023,   "alpha",0,       "theta", -pi / 4 + 0.959931));

EE = Base * Shoulder * Elbow

% Visualize in Blender
bpy.data.objects["EE"].matrix_world = Matrix([
  [],
  [],
  [],
  [0,0,0,1]
])
