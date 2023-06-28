shoulder = -0.631809;
bicep = 0.624828;
elbow = -0.820305;
forearm = 0.541052;
wrist = -1.281072;
palm = 0;

Shoulder = createTranslation3d(0, 0, 0.203) * createRotationOz(shoulder);
Bicep = createTranslation3d(0.075, 0.0735, 0.13) * createRotationOy(bicep);
Elbow = createTranslation3d(0, -0.0055, 0.27) * createRotationOy(elbow);
Forearm = createTranslation3d(0.106, -0.068001, 0.09) * createRotationOx(forearm);
Wrist = createTranslation3d(0.187, -0.029, 0) * createRotationOy(wrist);
Palm = createTranslation3d(0.052, 0.029, 0) * createRotationOx(palm);
Tool = createTranslation3d(0.03, 0, 0);

EE = Shoulder * Bicep * Elbow * Forearm * Wrist * Palm * Tool

% Position
[EE(1,4), EE(2,4), EE(3,4)]

% Orientation
rotation3dToEulerAngles(EE)

rotation3dToEulerAngles([...
   [0.743628,    0.247372,   -0.621147,    0.553711];
  [-0.589392,   -0.196065,   -0.783694,   -0.409518];
  [-0.315649,    0.948876,   6.12323e-17,   -0.130246];
  [        0,           0,           0,           1];
]);

% with striker

base = 1.230530;
shoulder = 0.211682;
elbow = -1.926937;

Base = ...
  dh(struct("a", 0,       "d", 0.11518, "alpha", pi / 2, "theta", base)) * ...
  dh(struct("a", -0.013,  "d", 0,       "alpha",0,       "theta", 0));
Shoulder = ...
  dh(struct("a", 0.4173,  "d", 0,       "alpha",0,       "theta", shoulder));
Elbow = ...
  dh(struct("a", 0.48059, "d", 0,       "alpha",0,       "theta", elbow)) * ...
  dh(struct("a", 0.008,   "d", 0,       "alpha",0,       "theta", pi / 4)) * ...
  dh(struct("a", 0.295,   "d",-0.023,   "alpha",0,       "theta", -pi / 4 + 0.959931));

EE = Base * Shoulder * Elbow

rotation3dToEulerAngles(EE, "XYZ")

% orientation has X pointing down the drumstick

% test 2 with striker

base = 1.230530;
shoulder = 0.211682;
elbow = -0.558599;

% This is not working!
% The position is correct but Orientation X is not pointing down arm
% I guess this is the part I haven't figured out yet!

% Here's what we get

Orientation.pitch = 0.9578
Orientation.yaw = -1.9111

Normal.pitch = 0.6130
Normal.yaw = 1.2305

Approach.pitch = 0
Approach.yaw = -0.3402

% Here are the angles we get from Blender

P = [ 0.344232; 1.04122; 0.214933 ];
O.roll = 1.570796;  % pi / 2
O.pitch = -0.613014;
O.yaw = 1.230530;

% We get this from math toolbox
O.roll = 1.570796;  % pi / 2, this matches
O.pitch = 1.23052694; % looks like the yaw from Blender??
O.yaw = 0.613011993; % looks like the pitch from Blender??

% Why are these two angles switched up like that?
% What if this was the problem all along?

% Could this be the Xrot90 we do in the beginning?
% Has to be.

% Final Z axis = Y axis
% Final Y axis = Z axis

% One more time, now with the knowledge that the axis are switched YZ

base = -0.568977;
shoulder = 1.2868;
elbow = -0.272365;

Base = ...
  dh(struct("a", 0,       "d", 0.11518, "alpha", pi / 2, "theta", base)) * ...
  dh(struct("a", -0.013,  "d", 0,       "alpha",0,       "theta", 0));
Shoulder = ...
  dh(struct("a", 0.4173,  "d", 0,       "alpha",0,       "theta", shoulder));
Elbow = ...
  dh(struct("a", 0.48059, "d", 0,       "alpha",0,       "theta", elbow)) * ...
  dh(struct("a", 0.008,   "d", 0,       "alpha",0,       "theta", pi / 4)) * ...
  dh(struct("a", 0.295,   "d",-0.023,   "alpha",0,       "theta", -pi / 4 + 0.959931));

EE = Base * Shoulder * Elbow

rotation3dToEulerAngles(EE, "XYZ")

% 90.000   -32.600   113.123
% Blender got
% 90.000  -113.123   -23.6

% This time they match but the sign of yaw is wrong... Looks like it's always wrong the same way though??

Roll = X
Pitch = -Z
Yaw = Y

% And one more time

base = -0.568977;
shoulder = 1.2868;
elbow = -1.22182;

Base = ...
  dh(struct("a", 0,       "d", 0.11518, "alpha", pi / 2, "theta", base)) * ...
  dh(struct("a", -0.013,  "d", 0,       "alpha",0,       "theta", 0));
Shoulder = ...
  dh(struct("a", 0.4173,  "d", 0,       "alpha",0,       "theta", shoulder));
Elbow = ...
  dh(struct("a", 0.48059, "d", 0,       "alpha",0,       "theta", elbow)) * ...
  dh(struct("a", 0.008,   "d", 0,       "alpha",0,       "theta", pi / 4)) * ...
  dh(struct("a", 0.295,   "d",-0.023,   "alpha",0,       "theta", -pi / 4 + 0.959931));

EE = Base * Shoulder * Elbow

rotation3dToEulerAngles(EE, "XYZ")

% OK, this is solid with striker
% I wonder if we can multiply EE by something to get angles we put into Blender?

% Back to kuka

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
bpy.data.objects["EE"].matrix_world = Matrix()

% visualize a vector
N = [ bpy.data.objects["EE"].location + Vector([0.0809, 0.4877, -0.8693]) * 0.2, bpy.data.objects["EE"].location ]
O = [ bpy.data.objects["EE"].location + Vector([0.8317, 0.4477, 0.3286]) * 0.2, bpy.data.objects["EE"].location ]
A = [ bpy.data.objects["EE"].location + Vector([0.5494, -0.7495, -0.3694]) * 0.2, bpy.data.objects["EE"].location ]

% striker

base = 0;
shoulder = 0;
elbow = 0;

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
  createTranslation3d(0.48059, 0, -0.023) * ...
  createRotationOz(0.959931) * ...
  createTranslation3d(0.305, 0, 0);

% createTranslation3d(0.295, 0, 0);

EE = Base * Shoulder * Elbow;