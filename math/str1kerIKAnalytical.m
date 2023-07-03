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

% Simplified equations for display

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

% TODO: expand these, solve won't work

% E3 gets us theta1
theta1Solution = solve(E3, theta1)(1,1);

% E4 gets us theta2, since theta3 cancels out with pythagorean identity
E4 = subs(E4, theta1, theta1Solution);
theta2Solution = solve(E4, theta2)(1,1);

% E1 gets us theta3
E1 = subs(E1, theta1, theta1Solution);
E1 = subs(E1, theta2, theta2Solution);
theta3Solution = solve(E1, theta3)(1,1);

base = vpa(theta1Solution)
shoulder = vpa(theta2Solution)
elbow = vpa(theta3Solution)
