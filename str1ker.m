syms theta1 theta2 theta3 real

Base      = dh(struct("a", 0,       "d", 0.11518, "alpha", pi / 2, "theta", theta1));
Offset    = dh(struct("a", -0.013,  "d", 0,       "alpha",0,       "theta", 0));
Shoulder  = dh(struct("a", 0.4173,  "d", 0,       "alpha",0,       "theta", theta2));
Elbow     = dh(struct("a", 0.48059, "d", 0,       "alpha",0,       "theta", theta3));
Wrist     = dh(struct("a", 0.008,   "d", 0,       "alpha",0,       "theta", pi / 4));
Drumstick = dh(struct("a", 0.295,   "d",-0.023,   "alpha",0,       "theta", -pi / 4 + 0.959931));

% Prove that yaw of Orientation is yaw of Approach + pi / 2

% Test Case 1

theta11 = 1.230530;
theta12 = 0.211682;
theta13 = -1.926937;

Base1 =      dh(a = 0,         d = 0.11518,  alpha = pi / 2,  theta = theta11);
Offset1 =    dh(a = -0.013,    d = 0,        alpha = 0,       theta = 0);
Shoulder1 =  dh(a = 0.4173,    d = 0,        alpha = 0,       theta = theta12);
Elbow1 =     dh(a = 0.48059,   d = 0,        alpha = 0,       theta = theta13);
Wrist1 =     dh(a = 0.008,     d = 0,        alpha = 0,       theta = pi / 4);
Drumstick1 = dh(a = 0.295,     d = -0.023,   alpha = 0,       theta = -pi / 4 + 0.959931);

Effector1 = Base1 * Offset1 * Shoulder1 * Elbow1 * Wrist1 * Drumstick1;

N1 = [ Effector1(1,1); Effector1(2,1); Effector1(3,1); ];
O1 = [ Effector1(1,2); Effector1(2,2); Effector1(3,2); ];
A1 = [ Effector1(1,3); Effector1(2,3); Effector1(3,3); ];
P1 = [ Effector1(1,4); Effector1(2,4); Effector1(3,4); ];

O1angles = eulerAngles(O1);
A1angles = eulerAngles(A1);
N1angles = eulerAngles(N1);

% Test Case 2

theta21 = -0.652804;
theta22 = 1.239454;
theta23 = -0.958185;

Base2 =      dh(a = 0,         d = 0.11518,  alpha = pi / 2,  theta =-0.652804);
Offset2 =    dh(a = -0.013,    d = 0,        alpha = 0,       theta = 0);
Shoulder2 =  dh(a = 0.4173,    d = 0,        alpha = 0,       theta = 1.239454);
Elbow2 =     dh(a = 0.48059,   d = 0,        alpha = 0,       theta = -0.958185);
Wrist2 =     dh(a = 0.008,     d = 0,        alpha = 0,       theta = pi / 4);
Drumstick2 = dh(a = 0.295,     d = -0.023,   alpha = 0,       theta = -pi / 4 + 0.959931);

Effector2 = Base2 * Offset2 * Shoulder2 * Elbow2 * Wrist2 * Drumstick2;

N2 = [ Effector2(1,1); Effector2(2,1); Effector2(3,1); ];
O2 = [ Effector2(1,2); Effector2(2,2); Effector2(3,2); ];
A2 = [ Effector2(1,3); Effector2(2,3); Effector2(3,3); ];
P2 = [ Effector2(1,4); Effector2(2,4); Effector2(3,4); ];

O2angles = eulerAngles(O2);
A2angles = eulerAngles(A2);
N2angles = eulerAngles(N2);

% Test Case 3

theta31 = -2.325364;
theta32 = 0.298671;
theta33 = -2.209587;

Base3 =      dh(a = 0,         d = 0.11518,  alpha = pi / 2,  theta = -2.325364);
Offset3 =    dh(a = -0.013,    d = 0,        alpha = 0,       theta = 0);
Shoulder3 =  dh(a = 0.4173,    d = 0,        alpha = 0,       theta = 0.298671);
Elbow3 =     dh(a = 0.48059,   d = 0,        alpha = 0,       theta = -2.209587);
Wrist3 =     dh(a = 0.008,     d = 0,        alpha = 0,       theta = pi / 4);
Drumstick3 = dh(a = 0.295,     d = -0.023,   alpha = 0,       theta = -pi / 4 + 0.959931);
Effector3 = Base3 * Offset3 * Shoulder3 * Elbow3 * Wrist3 * Drumstick3;

N3 = [ Effector3(1,1); Effector3(2,1); Effector3(3,1); ];
O3 = [ Effector3(1,2); Effector3(2,2); Effector3(3,2); ];
A3 = [ Effector3(1,3); Effector3(2,3); Effector3(3,3); ];
P3 = [ Effector3(1,4); Effector3(2,4); Effector3(3,4); ];

O3angles = eulerAngles(O3);
A3angles = eulerAngles(A3);
N3angles = eulerAngles(N3);

% Test Case 4

theta41 = 2.279625;
theta42 = 0.875178;
theta43 = -0.798914;

Base4 =      dh(a = 0,         d = 0.11518,  alpha = pi / 2,  theta = 2.279625);
Offset4 =    dh(a = -0.013,    d = 0,        alpha = 0,       theta = 0);
Shoulder4 =  dh(a = 0.4173,    d = 0,        alpha = 0,       theta = 0.875178);
Elbow4 =     dh(a = 0.48059,   d = 0,        alpha = 0,       theta = -0.798914);
Wrist4 =     dh(a = 0.008,     d = 0,        alpha = 0,       theta = pi / 4);
Drumstick4 = dh(a = 0.295,     d = -0.023,   alpha = 0,       theta = -pi / 4 + 0.959931);
Effector4 = Base4 * Offset4 * Shoulder4 * Elbow4 * Wrist4 * Drumstick4;

N4 = [ Effector4(1,1); Effector4(2,1); Effector4(3,1); ];
O4 = [ Effector4(1,2); Effector4(2,2); Effector4(3,2); ];
A4 = [ Effector4(1,3); Effector4(2,3); Effector4(3,3); ];
P4 = [ Effector4(1,4); Effector4(2,4); Effector4(3,4); ];

round(P4,4)
O4angles = eulerAngles(O4);
A4angles = eulerAngles(A4);
N4angles = eulerAngles(N4);

% Test Case 5

theta51 = 0.249411;
theta52 = 0.875178;
theta53 = -0.798914;

Base5 =      dh(a = 0,         d = 0.11518,  alpha = pi / 2,  theta = 0.249411);
Offset5 =    dh(a = -0.013,    d = 0,        alpha = 0,       theta = 0);
Shoulder5 =  dh(a = 0.4173,    d = 0,        alpha = 0,       theta = 0.875178);
Elbow5 =     dh(a = 0.48059,   d = 0,        alpha = 0,       theta = -0.798914);
Wrist5 =     dh(a = 0.008,     d = 0,        alpha = 0,       theta = pi / 4);
Drumstick5 = dh(a = 0.295,     d = -0.023,   alpha = 0,       theta = -pi / 4 + 0.959931);
Effector5 = Base5 * Offset5 * Shoulder5 * Elbow5 * Wrist5 * Drumstick5;

N5 = [ Effector5(1,1); Effector5(2,1); Effector5(3,1); ];
O5 = [ Effector5(1,2); Effector5(2,2); Effector5(3,2); ];
A5 = [ Effector5(1,3); Effector5(2,3); Effector5(3,3); ];
P5 = [ Effector5(1,4); Effector5(2,4); Effector5(3,4); ];

round(P5,4)
O5angles = eulerAngles(O5);
A5angles = eulerAngles(A5);
N5angles = eulerAngles(N5);

% Test Case 6 - Copy of 1 but with arm extended up

theta61 = 1.230530;
theta62 = 0.755362;
theta63 = -0.946005;

Base6 =      dh(a = 0,         d = 0.11518,  alpha = pi / 2,  theta = theta61);
Offset6 =    dh(a = -0.013,    d = 0,        alpha = 0,       theta = 0);
Shoulder6 =  dh(a = 0.4173,    d = 0,        alpha = 0,       theta = theta62);
Elbow6 =     dh(a = 0.48059,   d = 0,        alpha = 0,       theta = theta63);
Wrist6 =     dh(a = 0.008,     d = 0,        alpha = 0,       theta = pi / 4);
Drumstick6 = dh(a = 0.295,     d = -0.023,   alpha = 0,       theta = -pi / 4 + 0.959931);
Effector6 = Base6 * Offset6 * Shoulder6 * Elbow6 * Wrist6 * Drumstick6;

N6 = [ Effector6(1,1); Effector6(2,1); Effector6(3,1); ];
O6 = [ Effector6(1,2); Effector6(2,2); Effector6(3,2); ];
A6 = [ Effector6(1,3); Effector6(2,3); Effector6(3,3); ];
P6 = [ Effector6(1,4); Effector6(2,4); Effector6(3,4); ];

round(P6,4)
O6angles = eulerAngles(O6);
A6angles = eulerAngles(A6);
N6angles = eulerAngles(N6);

