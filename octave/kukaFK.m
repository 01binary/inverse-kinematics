% Forward kinematics
pkg load matgeom

shoulder = -0.53058;
bicep = 0.949459;
elbow = 0.031416;
forearm = 0.631809;
wrist = 0.897099;
palm = 0;

Shoulder = createTranslation3d(0, 0, 0.203) * createRotationOz(shoulder);
Bicep = createTranslation3d(0.075, 0.0735, 0.13) * createRotationOy(bicep);
Elbow = createTranslation3d(0, -0.0055, 0.27) * createRotationOy(elbow);
Forearm = createTranslation3d(0.106, -0.068001, 0.09) * createRotationOx(forearm);
Wrist = createTranslation3d(0.187, -0.029, 0) * createRotationOy(wrist);
Palm = createTranslation3d(0.052, 0.029, 0) * createRotationOx(palm);
Tool = createTranslation3d(0.03, 0, 0);

EE = Shoulder * Bicep * Elbow * Forearm * Wrist * Palm * Tool

% visualize a matrix
% bpy.data.objects["EE"].matrix_world = Matrix()

% visualize a vector
% N = [ bpy.data.objects["EE"].location + Vector([0.0809, 0.4877, -0.8693]) * 0.2, bpy.data.objects["EE"].location ]
% O = [ bpy.data.objects["EE"].location + Vector([0.8317, 0.4477, 0.3286]) * 0.2, bpy.data.objects["EE"].location ]
% A = [ bpy.data.objects["EE"].location + Vector([0.5494, -0.7495, -0.3694]) * 0.2, bpy.data.objects["EE"].location ]