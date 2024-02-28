% Analytical Inverse Kinematics with Symbols (Matlab)

% Define joint Variables as Symbols
% If you don't declare them as "real", you may get imaginary numbers
syms base shoulder elbow wrist real

% Define Goal as Symbols
% If you don't declare them as "real", you may get imaginary numbers
syms nx ny nz ox oy oz ax ay az px py pz real

EE = [ ...
  % Normal, Orientation, Approach, Position (NOAP)
  [ nx, ox, ax, px ];
  [ ny, oy, ay, py ];
  [ nz, oz, az, pz ];
  [ 0,  0,  0,  1  ];
];

% Define Joint Frames
Base = zrotate(base);
Shoulder = ...
  translate(0, 0, 0.670) * ...
  yrotate(shoulder);
Elbow = ...
  translate(0.7, 0, 0) * ...
  yrotate(elbow);
Wrist = ...
  translate(0.7, 0.05, 0) * ...
  xrotate(wrist);
Tool = ...
  translate(0.18, 0, 0);

% Why is this not working?
% The IK solution for Str1ker worked!

% Solve for wrist joint variable and substitute into Wrist frame
% Rotation along X axis of the goal pose
Wrist == EE
sinWrist = oz;
cosWrist = oy;
wristSolution = atan2(sinWrist, cosWrist);
Wrist = subs(Wrist, wrist, wristSolution);



LHS = Base * Shoulder;
RHS = EE * inv(Tool) * inv(Wrist) * inv(Elbow);

% System of Nonlinear Equations from IK Equation
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

% Look for solutions for each variable
% For each equation E1...E12
% isolate(En, trigfn(jointVariable))
% Then substitute into the next equation using subs()

% For example...

sinBase = rhs(isolate(E2, sin(base)))
cosBase = rhs(isolate(E6, cos(base)))
base = atan2(sinBase, cosBase)

