% 1. Derive Approach from Base Position and Target

% Using Example 1
Target = [ 0.1603; 0.5218; -0.4814 ];
BasePosition = [ 0; 0; 0 ];

% Base position
bx = BasePosition(1,1);
by = BasePosition(2,1);
bz = BasePosition(3,1);

% Target position
tx = Target(1,1);
ty = Target(2,1);

drumstickOffset = -0.023;
angleToTarget = atan2(ty - by, tx - bx); % 1.2727
distanceToTarget = sqrt((tx - bx)^2 + (ty - by)^2); % 0.5458
angleWristOffset = abs(asin(drumstickOffset / distanceToTarget)); % 0.0421
baseAngle = angleToTarget - angleWristOffset; % 1.2305
approachYawAngle = baseAngle - pi / 2; % -0.3403

Aguess = [
  cos(approachYawAngle);
  sin(approachYawAngle);
  0
];
% [ 0.9427; -0.3337; 0]

% 2. Derive Normal from Approach, Shoulder Position, and Target

% Algebraic solution for Normal yaw

normalYawAngle = approachYawAngle + pi / 2; % 1.2305

% Geometric solution for Normal Z component

% Get shoulder position by running FK with first two frames
B = dh(a = 0,         d = 0.11518,  alpha = pi / 2,  theta = baseAngle);
O = dh(a = -0.013,    d = 0,        alpha = 0,       theta = 0);
ShoulderPosition = B * O * [bx; by; bz; 1];
% [ -0.0043; -0.0123; 0.1152 ]

% Shoulder position
rx = ShoulderPosition(1,1);
ry = ShoulderPosition(2,1);
rz = ShoulderPosition(3,1);

% Target position
tx = Target(1,1);
ty = Target(2,1);
tz = Target(3,1);

normalPitchAngle = atan(sqrt((tx - rx)^2 + (ty - ry)^2) / (tz - rz)) % -0.7528

Nguess = [
  cos(normalYawAngle) * cos(normalPitchAngle);
  sin(normalYawAngle) * cos(normalPitchAngle);
  sin(normalPitchAngle)
];
% [ 0.2436; 0.688; -0.6837 ]

% 3. Derive Orientation from Approach and Normal

Oguess = cross(Aguess, Nguess);
% [ 0.2282; 0.6445; 0.7298 ]

% 4. Assemble the End Effector matrix

EEguess = [
  Nguess(1,1), Oguess(1,1), Aguess(1,1), Target(1,1);
  Nguess(2,1), Oguess(2,1), Aguess(2,1), Target(2,1);
  Nguess(3,1), Oguess(3,1), Aguess(3,1), Target(3,1);
  0, 0, 0, 1
];
% [ 0.2436, 0.2282,  0.9427,  0.1603]
% [  0.688, 0.6445, -0.3337,  0.5218]
% [-0.6837, 0.7298,       0, -0.4814]
% [      0,      0,       0,     1.0]

% 5. Solve for the joint angles using FK equation

syms theta2 theta3 real

Base = ...
  dh(struct("a", 0,       "d", 0.11518, "alpha", pi / 2, "theta", baseAngle)) * ...
  dh(struct("a", -0.013,  "d", 0,       "alpha",0,       "theta", 0));
Shoulder = ...
  dh(struct("a", 0.4173,  "d", 0,       "alpha",0,       "theta", theta2));
Elbow = ...
  dh(struct("a", 0.48059, "d", 0,       "alpha",0,       "theta", theta3)) * ...
  dh(struct("a", 0.008,   "d", 0,       "alpha",0,       "theta", pi / 4)) * ...
  dh(struct("a", 0.295,   "d",-0.023,   "alpha",0,       "theta", -pi / 4 + 0.959931));

% Base is already known since we solved for baseAngle geometrically

IK = Base * Shoulder * Elbow == EEguess;
LHS = lhs(IK);
RHS = rhs(IK);

% Equations

E1 = vpa(LHS(1,1) == RHS(1,1));
E2 = vpa(LHS(1,2) == RHS(1,2));
E3 = vpa(LHS(1,3) == RHS(1,3));
E4 = vpa(LHS(1,4) == RHS(1,4));

E5 = vpa(LHS(2,1) == RHS(2,1));
E6 = vpa(LHS(2,2) == RHS(2,2));
E7 = vpa(LHS(2,3) == RHS(2,3));
E8 = vpa(LHS(2,4) == RHS(2,4));

E9 = vpa(LHS(3,1) == RHS(3,1));
E10 = vpa(LHS(3,2) == RHS(3,2));
E11 = vpa(LHS(3,3) == RHS(3,3));
E12 = vpa(LHS(3,4) == RHS(3,4));

E13 = vpa(LHS(4,1) == RHS(4,1));
E14 = vpa(LHS(4,2) == RHS(4,2));
E15 = vpa(LHS(4,3) == RHS(4,3));
E16 = vpa(LHS(4,4) == RHS(4,4));

% As we can see, theta2 and theta3 are always coupled
% Decouple theta3 (elbow) from theta2 (shoulder)

IK = Base * Shoulder * Elbow * inv(Elbow) == EEguess * inv(Elbow);

IK = Base * Shoulder == EEguess * inv(Elbow);

LHS = lhs(IK);
RHS = rhs(IK);

% Equations

E1 = LHS(1,1) == RHS(1,1);
E2 = LHS(1,2) == RHS(1,2);
E3 = LHS(1,3) == RHS(1,3);
E4 = LHS(1,4) == RHS(1,4);

E5 = LHS(2,1) == RHS(2,1);
E6 = LHS(2,2) == RHS(2,2);
E7 = LHS(2,3) == RHS(2,3);
E8 = LHS(2,4) == RHS(2,4);

E9 = LHS(3,1) == RHS(3,1);
E10 = LHS(3,2) == RHS(3,2);
E11 = LHS(3,3) == RHS(3,3);
E12 = LHS(3,4) == RHS(3,4);

E13 = LHS(4,1) == RHS(4,1);
E14 = LHS(4,2) == RHS(4,2);
E15 = LHS(4,3) == RHS(4,3);
E16 = LHS(4,4) == RHS(4,4);

% E4 gives us theta2

solve(E4, theta2)
% theta2 = +- 0.23079420855329669970264920936401
% elbow up would be positive, elbow down would be negative
% since we know our robot is fixed to elbow up configuraiton, we choose positive

% E1 gives us theta3 in terms of theta2

E1 = subs(E1, 'theta2', 0.2308)
solve(E1, theta3)
% theta3 = -1.9434964917358216317824910682065

% Compare to Test Case 1

theta11 = 1.230530;
theta12 = 0.211682;
theta13 = -1.926937;

theta1 == baseAngle == 1.2305
theta2 == 0.23079420855329669970264920936401
theta3 == -1.926937

% The match is close enough (within 0.1 degrees)