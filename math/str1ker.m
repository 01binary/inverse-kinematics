pkg load symbolic
syms theta1 theta2 theta3 real

EE = ...
  % base
  dh(struct("a", 0,       "d", 0.11518, "alpha", pi / 2, "theta", theta1)) * ...
  dh(struct("a", -0.013,  "d", 0,       "alpha",0,       "theta", 0)) * ...
  % shoulder
  dh(struct("a", 0.4173,  "d", 0,       "alpha",0,       "theta", theta2)) * ...
  % elbow
  dh(struct("a", 0.48059, "d", 0,       "alpha",0,       "theta", theta3)) * ...
  dh(struct("a", 0.008,   "d", 0,       "alpha",0,       "theta", pi / 4)) * ...
  dh(struct("a", 0.295,   "d",-0.023,   "alpha",0,       "theta", -pi / 4 + 0.959931))


% Prove that yaw of Orientation is yaw of Approach + pi / 2

% Test Case 1

theta11 = 1.230530;
theta12 = 0.211682;
theta13 = -1.926937;

EE1 = fk(theta11, theta12, theta13);

N1 = [ EE1(1,1); EE1(2,1); EE1(3,1); ];
O1 = [ EE1(1,2); EE1(2,2); EE1(3,2); ];
A1 = [ EE1(1,3); EE1(2,3); EE1(3,3); ];
P1 = [ EE1(1,4); EE1(2,4); EE1(3,4); ];

O1angles = eu(O1);
A1angles = eu(A1);
N1angles = eu(N1);

% Test Case 2

theta21 = -0.652804;
theta22 = 1.239454;
theta23 = -0.958185;

EE2 = fk(theta21, theta22, theta23);

N2 = [ EE2(1,1); EE2(2,1); EE2(3,1); ];
O2 = [ EE2(1,2); EE2(2,2); EE2(3,2); ];
A2 = [ EE2(1,3); EE2(2,3); EE2(3,3); ];
P2 = [ EE2(1,4); EE2(2,4); EE2(3,4); ];

O2angles = eu(O2);
A2angles = eu(A2);
N2angles = eu(N2);

% Test Case 3

theta31 = -2.325364;
theta32 = 0.298671;
theta33 = -2.209587;

EE3 = fk(theta31, theta32, theta33);

N3 = [ EE3(1,1); EE3(2,1); EE3(3,1); ];
O3 = [ EE3(1,2); EE3(2,2); EE3(3,2); ];
A3 = [ EE3(1,3); EE3(2,3); EE3(3,3); ];
P3 = [ EE3(1,4); EE3(2,4); EE3(3,4); ];

O3angles = eu(O3);
A3angles = eu(A3);
N3angles = eu(N3);

% Test Case 4

theta41 = 2.279625;
theta42 = 0.875178;
theta43 = -0.798914;

EE4 = fk(theta41, theta42, theta43);

N4 = [ EE4(1,1); EE4(2,1); EE4(3,1); ];
O4 = [ EE4(1,2); EE4(2,2); EE4(3,2); ];
A4 = [ EE4(1,3); EE4(2,3); EE4(3,3); ];
P4 = [ EE4(1,4); EE4(2,4); EE4(3,4); ];

round(P4,4)
O4angles = eu(O4);
A4angles = eu(A4);
N4angles = eu(N4);

% Test Case 5

theta51 = 0.249411;
theta52 = 0.875178;
theta53 = -0.798914;

EE5 = fk(theta51, theta52, theta53);

N5 = [ EE5(1,1); EE5(2,1); EE5(3,1); ];
O5 = [ EE5(1,2); EE5(2,2); EE5(3,2); ];
A5 = [ EE5(1,3); EE5(2,3); EE5(3,3); ];
P5 = [ EE5(1,4); EE5(2,4); EE5(3,4); ];

round(P5,4)
O5angles = eu(O5);
A5angles = eu(A5);
N5angles = eu(N5);

% Test Case 6 - Copy of 1 but with arm extended up

theta61 = 1.230530;
theta62 = 0.755362;
theta63 = -0.946005;

EE6 = fk(theta61, theta62, theta63);

N6 = [ EE6(1,1); EE6(2,1); EE6(3,1); ];
O6 = [ EE6(1,2); EE6(2,2); EE6(3,2); ];
A6 = [ EE6(1,3); EE6(2,3); EE6(3,3); ];
P6 = [ EE6(1,4); EE6(2,4); EE6(3,4); ];

round(P6,4)
O6angles = eu(O6);
A6angles = eu(A6);
N6angles = eu(N6);

