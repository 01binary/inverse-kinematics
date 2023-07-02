base = 0.4;
shoulder = 0.8;
elbow = -0.6;

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

EE = Base * Shoulder * Elbow;

% visualize a matrix
bpy.data.objects["EE"].matrix_world = Matrix()

% visualize a vector
N = [ bpy.data.objects["EE"].location + Vector([0.3679, 0.1555, 0.9168]) * 0.2, bpy.data.objects["EE"].location ]
O = [ bpy.data.objects["EE"].location + Vector([-0.8444, -0.3570, 0.3994]) * 0.2, bpy.data.objects["EE"].location ]
A = [ bpy.data.objects["EE"].location + Vector([0.3894,  -0.9211, 0]) * 0.2, bpy.data.objects["EE"].location ]

% Can I do IK without knowing EE pitch, only yaw?

Reference = [
  [0.3679, -0.8444, 0.3894, 0.7929];
  [0.1555, -0.3570, -0.9211, 0.3602];
  [0.9168, 0.3994, 0.0000, 0.7896];
  [0, 0, 0, 1.0000];
]

% Origin
Origin = [ 0; 0; 0 ];
baseX = Origin(1,1);
baseY = Origin(2,1);
baseZ = Origin(3,1);

% Goal
Goal = [ Reference(1,4); Reference(2,4); Reference(3,4) ];
goalX = Goal(1,1);
goalY = Goal(2,1);

% Approach
drumstickOffset = -0.023;
angleToTarget = atan2(goalY - baseY, goalX - baseX);
distanceToTarget = sqrt((goalX - baseX)^2 + (goalY - baseY)^2);
angleWristOffset = abs(asin(drumstickOffset / distanceToTarget));
baseAngle = angleToTarget - angleWristOffset;
approachYawAngle = baseAngle - pi / 2;
Approach = [
  cos(approachYawAngle);
  sin(approachYawAngle);
  0
];

% Normal
syms endEffectorPitch real
normalYawAngle = baseAngle;
Normal = [
  cos(normalYawAngle) * cos(endEffectorPitch);
  sin(normalYawAngle) * cos(endEffectorPitch);
  sin(endEffectorPitch);
];

% Orientation
Orientation = cross(Approach, Normal);

% EE
EE = [ ...
  [ Normal(1,1), Orientation(1,1), Approach(1,1), Goal(1,1) ];
  [ Normal(2,1), Orientation(2,1), Approach(2,1), Goal(2,1) ];
  [ Normal(3,1), Orientation(3,1), Approach(3,1), Goal(3,1) ];
  [ 0, 0, 0, 1 ]
];

% Setup IK with theta1 known
theta1 = angleToTarget;
syms theta2 theta3 real

Base = ...
  dh(struct("a", 0,       "d", 0.11518, "alpha", pi / 2, "theta", theta1)) * ...
  dh(struct("a", -0.013,  "d", 0,       "alpha",0,       "theta", 0));
Shoulder = ...
  dh(struct("a", 0.4173,  "d", 0,       "alpha",0,       "theta", theta2));
Elbow = ...
  dh(struct("a", 0.48059, "d", 0,       "alpha",0,       "theta", theta3)) * ...
  dh(struct("a", 0.008,   "d", 0,       "alpha",0,       "theta", pi / 4)) * ...
  dh(struct("a", 0.295,   "d",-0.023,   "alpha",0,       "theta", -pi / 4 + 0.959931));

% Decouple Shoulder from Elbow

% Base * Shoulder * Elbow == EE
% Base * Shoulder * Elbow * inv(Elbow) == EE * inv(Elbow)
% IK = Base * Shoulder == EE * inv(Elbow);
LHS = Base * Shoulder;
RHS = EE * inv(Elbow);

% Equations

E1 = vpa(simplify(LHS(1,1) == RHS(1,1)));
E2 = vpa(simplify(LHS(1,2) == RHS(1,2)));
E3 = vpa(simplify(LHS(1,3) == RHS(1,3)));
E4 = vpa(simplify(LHS(1,4) == RHS(1,4)));

E5 = vpa(simplify(LHS(2,1) == RHS(2,1)));
E6 = vpa(simplify(LHS(2,2) == RHS(2,2)));
E7 = vpa(simplify(LHS(2,3) == RHS(2,3)));
E8 = vpa(simplify(LHS(2,4) == RHS(2,4)));

E9 = vpa(simplify(LHS(3,1) == RHS(3,1)));
E10 = vpa(simplify(LHS(3,2) == RHS(3,2)));
E11 = vpa(simplify(LHS(3,3) == RHS(3,3)));
E12 = vpa(simplify(LHS(3,4) == RHS(3,4)));

% E4 gets us theta2, since theta3 cancels out
%theta2 is -0.23105701720126 or 0.23105701720126
% we pick negative value for elbow up (positive is elbow down)

solve(E4, theta2)

% substitute theta2 into any other equation to get theta3

% OK, so giving up on getting a position-only solution with this approach???

