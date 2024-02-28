% Analytical Inverse Kinematics with Symbols (Matlab)

% Define joint Variables as Symbols
% If you don't declare them as "real", you may get imaginary numbers
syms base shoulder elbow real

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
  xrotate(0) * ...
  translate(0.18, 0, 0);

% Define IK Equation
IK = Base * Shoulder * Elbow * Wrist == EE;

% Decouple Wrist
IK = Base * Shoulder * Elbow == EE * inv(Wrist);

% Decouple Elbow
IK = Base * Shoulder == EE * inv(Wrist) * inv(Elbow);

% Define System of Nonlinear Equations from IK Equation
LHS = lhs(IK);
RHS = rhs(IK);

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

% What if we set wrist = 0 and solve, then back-track?

% E2, E6 expose base
S1 = rhs(isolate(E2, sin(base)))
C1 = rhs(isolate(E6, cos(base)))
baseSolution = atan2(S1, C1)

% E4, E8 expose elbow (two solutions)
C3a = rhs(isolate(E4, cos(elbow)))
C3b = rhs(isolate(E8, cos(elbow)))
elbowSolutions = solve([C3a == C3b], elbow)
elbowSolution = elbowSolutions(2,1)

% E1, E3 expose shoulder after substituting base and elbow
E1 = subs(E1, [base, elbow], [baseSolution, elbowSolution]);
E3 = subs(E3, [base, elbow], [baseSolution, elbowSolution]);
C2 = rhs(isolate(E1, cos(shoulder)))
S2 = rhs(isolate(E3, sin(shoulder)))
shoulderSolution = atan2(S2, C2)

% Substite shoulder, elbow, and base into IK equation to solve for wrist
syms wrist real

Base = zrotate(baseSolution);
Shoulder = ...
  translate(0, 0, 0.670) * ...
  yrotate(shoulderSolution);
Elbow = ...
  translate(0.7, 0, 0) * ...
  yrotate(elbowSolution);
Wrist = ...
  translate(0.7, 0.05, 0) * ...
  xrotate(wrist) * ...
  translate(0.18, 0, 0);

IK = Base * Shoulder * Elbow * Wrist == EE;

LHS = lhs(IK);
RHS = rhs(IK);

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

% E2, E3 expose wrist
C4a = rhs(isolate(E2, cos(wrist)))
C4b = rhs(isolate(E3, cos(wrist)))
wristSolutions = solve([C4a == C4b], wrist, 'Real',true)
wristSolution1 = wristSolutions(1,1)
wristSolution2 = wristSolutions(2,1)

% Convert to C++
ccode(baseSolution)
ccode(simplify(shoulderSolution))
ccode(simplify(elbowSolution))
ccode(wristSolution1)
ccode(wristSolution2)
