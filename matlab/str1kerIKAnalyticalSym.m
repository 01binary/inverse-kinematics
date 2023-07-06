% Analytical Inverse Kinematics with Symbols

syms theta1 theta2 theta3 real
syms nx ny nz ox oy oz ax ay az px py pz real

% Goal
EE = [ ...
  [ nx, ox, ax, px ];
  [ ny, oy, ay, py ];
  [ nz, oz, az, pz ];
  [ 0,  0,  0,  1  ];
];

% Joints
Base = ...
  dh(a = 0,       d = 0.11518, alpha = pi / 2,  theta = theta1) * ...
  dh(a = -0.013,  d = 0,       alpha = 0,       theta = 0);
Shoulder = ...
  dh(a = 0.4173,  d = 0,       alpha = 0,       theta = theta2);
Elbow = ...
  dh(a = 0.48059, d = 0,       alpha = 0,       theta = theta3) * ...
  dh(a = 0.008,   d = 0,       alpha = 0,       theta = pi / 4) * ...
  dh(a = 0.305,   d = -0.023,  alpha = 0,       theta = -pi / 4 + 0.959931);

% Equations
LHS = Base * Shoulder;
RHS = EE * inv(Elbow);

E1 = simplify(LHS(1,1) == RHS(1,1));
E2 = simplify(LHS(1,2) == RHS(1,2));
E3 = simplify(LHS(1,3) == RHS(1,3));
E4 = simplify(LHS(1,4) == RHS(1,4));

E5 = simplify(LHS(2,1) == RHS(2,1));
E6 = simplify(LHS(2,2) == RHS(2,2));
E7 = simplify(LHS(2,3) == RHS(2,3));
E8 = simplify(LHS(2,4) == RHS(2,4));

E9 = simplify(LHS(3,1) == RHS(3,1));
E10 = simplify(LHS(3,2) == RHS(3,2));
E11 = simplify(LHS(3,3) == RHS(3,3));
E12 = simplify(LHS(3,4) == RHS(3,4));

% Solve E3 for sin(theta1)
S1 = rhs(isolate(E3, sin(theta1)));

% Solve E7 for cos(theta1)
C1 = rhs(isolate(E7, cos(theta1)));

% Solve for theta1
base = atan2(S1, C1)

% Solve E4 for cos(theta2) substituting cos(theta1)
C2 = vpa(rhs(isolate(subs(E4, cos(theta1), C1), cos(theta2))));

% Solve E12 for sin(theta2)
S2 = vpa(rhs(isolate(E12, sin(theta2))));

% Solve for theta2
shoulder = atan2(S2, C2)

% Solve E2 for sin(theta3)
S3 = rhs(isolate(E2, sin(theta3)));

% Solve E10 for cos(theta3) substituting sin(theta3)
C3 = vpa(rhs(isolate(subs(E10, sin(theta3), S3), cos(theta3))));

% Solve E2 for sin(theta3) substituting cos(theta3)
S3 = vpa(rhs(isolate(subs(E2, cos(theta3), C3), sin(theta3))));

% Solve for theta3
elbow = atan2(S3, C3);