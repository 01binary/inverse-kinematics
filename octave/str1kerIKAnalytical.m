% Analytical Inverse kinematics

% Goal position and orientation (copied from FK with 0.4, 0.8, -0.6)
EE = [ ...
  [ 0.3679,  -0.8444,   0.3894,   0.7970 ];
  [ 0.1555,  -0.3570,  -0.9211,   0.3619 ];
  [ 0.9168,   0.3994,   0.0000,   0.7963 ];
  [      0,        0,        0,   1.0000 ];
];

% Symbols
pkg load symbolic
syms theta1 theta2 theta3 real

% Joints
Base = ...
  dh(struct("a", 0,       "d", 0.11518, "alpha", pi / 2, "theta", theta1)) * ...
  dh(struct("a", -0.013,  "d", 0,       "alpha",0,       "theta", 0));
Shoulder = ...
  dh(struct("a", 0.4173,  "d", 0,       "alpha",0,       "theta", theta2));
Elbow = ...
  dh(struct("a", 0.48059, "d", 0,       "alpha",0,       "theta", theta3)) * ...
  dh(struct("a", 0.008,   "d", 0,       "alpha",0,       "theta", pi / 4)) * ...
  dh(struct("a", 0.305,   "d",-0.023,   "alpha",0,       "theta", -pi / 4 + 0.959931));

% Decouple Shoulder from Elbow
Base * Shoulder * Elbow == EE;
Base * Shoulder * Elbow * inv(Elbow) == EE * inv(Elbow);
Base * Shoulder == EE * inv(Elbow);
LHS = Base * Shoulder;
RHS = EE * inv(Elbow);

% Raw equations
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

% Simplified equations
e1 = vpa(simplify(LHS(1,1) == RHS(1,1)));
e2 = vpa(simplify(LHS(1,2) == RHS(1,2)));
e3 = vpa(simplify(LHS(1,3) == RHS(1,3)));
e4 = vpa(simplify(LHS(1,4) == RHS(1,4)));

e5 = vpa(simplify(LHS(2,1) == RHS(2,1)));
e6 = vpa(simplify(LHS(2,2) == RHS(2,2)));
e7 = vpa(simplify(LHS(2,3) == RHS(2,3)));
e8 = vpa(simplify(LHS(2,4) == RHS(2,4)));

e9 = vpa(simplify(LHS(3,1) == RHS(3,1)));
e10 = vpa(simplify(LHS(3,2) == RHS(3,2)));
e11 = vpa(simplify(LHS(3,3) == RHS(3,3)));
e12 = vpa(simplify(LHS(3,4) == RHS(3,4)));

% Solve E3 for theta1
sin(theta1) == 0.38939999837912324949957017558898;
theta1Solution = asin(0.38939999837912324949957017558898);
% theta1Solution == 0.4

% Solve E4 for theta2 after substituting theta1
2.5552238333079459477319079853501e-17 * sin(theta1Solution) * sin(theta2) - 0.41729972022741453109509590183933 * cos(theta1Solution) * cos(theta2) + 0.013 * cos(theta1Solution) == -0.25584149093782381577760861692268;
-0.41729972022741453109509590183933 * cos(theta1Solution) * cos(theta2) + 0.013 * cos(theta1Solution) == -0.25584149093782381577760861692268;
-0.3844 * cos(theta2) + 0.011974 == -0.25584149093782381577760861692268;
-0.3844 * cos(theta2) == -0.2678;
cos(theta2) == -0.2678 / -0.3844;
theta2Solution = acos(0.6967);
% theta2Solution == 0.8001;

% Solve E2 for sin(theta3) after substituting theta1 and theta2
0.49999999999999996938383002131617 * sin(theta1Solution - theta2Solution) - 0.50000000000000003061616997868383 * sin(theta1Solution + theta2Solution) == 0.90271053312083078602703963509053 * sin(theta3) - 0.18296194884981026304932802370827 * cos(theta3);
-0.1947 - 0.4660 == 0.90271053312083078602703963509053 * sin(theta3) - 0.18296194884981026304932802370827 * cos(theta3);
-0.6607 == 0.90271053312083078602703963509053 * sin(theta3) - 0.18296194884981026304932802370827 * cos(theta3);
-0.6607 + 0.18296194884981026304932802370827 * cos(theta3) == 0.90271053312083078602703963509053 * sin(theta3);
(-0.6607 + 0.18296194884981026304932802370827 * cos(theta3)) / 0.90271053312083078602703963509053 == sin(theta3)

% Solve E10 for cos(theta3) after substituting theta1 and theta2
cos(theta2Solution) == 0.19868562757202296915693965121477 * sin(theta3) + 0.98008475703716721855798581315809 * cos(theta3);
0.6967 == 0.19868562757202296915693965121477 * sin(theta3) + 0.98008475703716721855798581315809 * cos(theta3);
0.6967 - 0.19868562757202296915693965121477 * sin(theta3) == 0.98008475703716721855798581315809 * cos(theta3);
(0.6967 - 0.19868562757202296915693965121477 * sin(theta3)) / 0.98008475703716721855798581315809 == cos(theta3);

% E2 and E10 give us a system of equations with sin and cos of theta3
sin(theta3) == (-0.6607 + 0.18296194884981026304932802370827 * cos(theta3)) / 0.90271053312083078602703963509053;
cos(theta3) == (0.6967 - 0.19868562757202296915693965121477 * sin(theta3)) / 0.98008475703716721855798581315809;

% Substitute cos(theta3) into sin(theta3) equation
sin(theta3) == (-0.6607 + 0.18296194884981026304932802370827 * ((0.6967 - 0.19868562757202296915693965121477 * sin(theta3)) / 0.98008475703716721855798581315809)) / 0.90271053312083078602703963509053;

% Simplify
sin(theta3) == -0.041087976585155759115400697353528 * sin(theta3) - 0.58782986886315897312781954985724;
1.041087976585155759115400697353528 * sin(theta3) == -0.58782986886315897312781954985724;
sin(theta3) == -0.58782986886315897312781954985724 / 1.041087976585155759115400697353528;
sin(theta3) == -0.5646;
theta3Solution = asin(-0.5646);
% theta3Solution == -0.5999

base = theta1Solution
shoulder = theta2Solution
elbow = theta3
