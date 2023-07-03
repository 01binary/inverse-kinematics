% Analytical Inverse kinematics

% Goal position and orientation (copied from FK with 0.4, 0.8, -0.6)
EE = [ ...
  [ 0.3679,  -0.8444,   0.3894,   0.7929 ];
  [ 0.1555,  -0.3570,  -0.9211,   0.3602 ];
  [ 0.9168,   0.3994,   0.0000,   0.7896 ];
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
2.5552238333079459477319079853501e-17 * sin(theta1Solution) * sin(theta2) - 0.41729972022741453109509590183933 * cos(theta1Solution) * cos(theta2) + 0.013 * cos(theta1Solution) == -0.25542063910970605643288257833867;
-0.41729972022741453109509590183933 * cos(theta1Solution) * cos(theta2) + 0.013 * cos(theta1Solution) == -0.25542063910970605643288257833867;
-0.41729972022741453109509590183933 * 0.9211 * cos(theta2) + 0.013 * 0.9211 == -0.25542063910970605643288257833867;
-0.41729972022741453109509590183933 * 0.9211 * cos(theta2) + 0.013 * 0.9211 == -0.25542063910970605643288257833867;
-0.3844 * cos(theta2) + 0.011974 * cos(theta2) == -0.25542063910970605643288257833867;
-0.3724 * cos(theta2) == -0.25542063910970605643288257833867;
cos(theta2) == -0.25542063910970605643288257833867 / -0.3724;
cos(theta2) == 0.6859;
theta2Solution = acos(0.6859);
% theta2Solution = 0.8150

% Could also solve E8 for theta2
0.41729972022741453109509590183933 * sin(theta1Solution) * cos(theta2) - 0.013 * sin(theta1Solution) + 2.5552238333079459477319079853501e-17 * sin(theta2) * cos(theta1Solution) == 0.10801451398093725527104962861025;
2.3535364633443558114348727806859e-17 * sin(theta2) + 0.16249651042985630557175542327021 * cos(theta2) - 0.0050621999789286022434944122826567 == 0.10801451187335092348284960422164;
0.16249651042985630557175542327021 * cos(theta2) - 0.0050621999789286022434944122826567 == 0.10801451187335092348284960422164;
cos(theta2) == (0.10801451187335092348284960422164 + 0.0050621999789286022434944122826567) / 0.16249651042985630557175542327021;
theta2Solution = acos((0.10801451187335092348284960422164 + 0.0050621999789286022434944122826567) / 0.16249651042985630557175542327021);
% theta2Solution = 0.8012

% Solve E2 for theta3 after substituting theta1 and theta2
0.49999999999999996938383002131617 * sin(theta1Solution - theta2Solution) - 0.50000000000000003061616997868383 * sin(theta1Solution + theta2Solution) == 0.90271053312083078602703963509053 * sin(theta3) - 0.18296194884981026304932802370827 * cos(theta3);
0.49999999999999996938383002131617 * sin(theta1Solution - theta2Solution) - 0.50000000000000003061616997868383 * sin(theta1Solution + theta2Solution) == 0.90271053312083078602703963509053 * sin(theta3) - 0.18296194884981026304932802370827 * cos(theta3);
-0.2016 - 0.4687 == 0.90271053312083078602703963509053 * sin(theta3) - 0.18296194884981026304932802370827 * cos(theta3);
-0.6703 == 0.90271053312083078602703963509053 * sin(theta3) - 0.18296194884981026304932802370827 * cos(theta3);

% Solve E10 for theta3 after substituting theta1 and theta2
cos(theta2Solution) == 0.19868562757202296915693965121477 * sin(theta3) + 0.98008475703716721855798581315809 * cos(theta3);
0.6859 == 0.19868562757202296915693965121477 * sin(theta3) + 0.98008475703716721855798581315809 * cos(theta3);

% E2 and E10 give us a system of equations
0.90271053312083078602703963509053 * sin(theta3) - 0.18296194884981026304932802370827 * cos(theta3) == -0.6703;
0.19868562757202296915693965121477 * sin(theta3) + 0.98008475703716721855798581315809 * cos(theta3) == 0.6859;

% Solve E2 for sin(theta3)
0.90271053312083078602703963509053 * sin(theta3) - 0.18296194884981026304932802370827 * cos(theta3) == -0.6703;
0.90271053312083078602703963509053 * sin(theta3) == 0.18296194884981026304932802370827 * cos(theta3) - 0.6703;
sin(theta3) == (0.18296194884981026304932802370827 * cos(theta3) - 0.6703) / 0.90271053312083078602703963509053;

% Substitute sin(theta3) into E10
0.19868562757202296915693965121477 * ((0.18296194884981026304932802370827 * cos(theta3) - 0.6703) / 0.90271053312083078602703963509053) + 0.98008475703716721855798581315809 * cos(theta3) == 0.6859;

% Simplify
1.0203543604103754240768344438625 * cos(theta3) - 0.14753217144165019166185129689679 == 0.68590004063388866314506298252743;
1.0203543604103754240768344438625 * cos(theta3) == 0.68590004063388866314506298252743 + 0.14753217144165019166185129689679;
cos(theta3) == 0.8334 / 1.0203543604103754240768344438625;
cos(theta3) == 0.8168
theta3Solution = acos(0.8168);

base = theta1Solution
shoulder = theta2Solution
elbow = theta3Solution
