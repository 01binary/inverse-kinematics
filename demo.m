% Plug in joint angles (from Blender) to calculate end effector position & orientation

Effector = fk(-1.143311, 0.6155, -2.5211)

% This is the result of calling the fk() function above with joint angles

Effector = [
    0.2426    0.3362   -0.9100    1.3954
   -0.5325   -0.7379   -0.4146   -2.6287
   -0.8109    0.5852    0.0000   -2.3121
         0         0         0    1.0000
];

% Now go in reverse (calculate joint angles from end effector position & orientation)
% The joint angles resoluting from solving the inverse equations should match what we started with

% Generate matrixes for each joint with theta1, theta2, and theta3 as unknowns

Base =      dh(a = 0,    d = 1,     alpha = pi / 2,  theta = theta1)
% This is a fixed offset matrix, not a joint
Offset =    dh(a = -0.2, d = 0,     alpha = 0,       theta = 0)
Shoulder =  dh(a = 3.5,  d = 0,     alpha = 0,       theta = theta2)
Elbow =     dh(a = 3.5,  d = 0,     alpha = 0,       theta = theta3)
% This is a fixed offset matrix, not a joint
Wrist =     dh(a = 2.5,  d = -0.18, alpha = 0,       theta = 0.959931)

% Solve the inverse kinematics equation for the three unknowns

Base * Offset * Shoulder * Elbow * Wrist == Effector

% Decouple Elbow from Base and Shoulder

Base * Offset * Shoulder * Elbow * Wrist * inv(Wrist) == Effector * inv(Wrist)

Base * Offset * Shoulder * Elbow == Effector * inv(Wrist)

Base * Offset * Shoulder * Elbow * inv(Elbow) == Effector * inv(Wrist) * inv(Elbow)

Base * Offset * Shoulder == Effector * inv(Wrist) * inv(Elbow)

% Alias LHS (left hand side of the equation) and RHS (right hand side)

LHS = Base * Offset * Shoulder

RHS = Effector * inv(Wrist) * inv(Elbow)

% Select (1,3) from LHS and RHS

sin(theta1) == -0.9100

% Solve for theta1 (Base)

theta1 = -1.1433

% Select (2,4) from LHS and RHS

(7*cos(theta2)*sin(-1.1433))/2 - sin(-1.1433)/5 == -2.4187

6557288672914079/36028797018963968 - (57376275887998193*cos(theta2))/18014398509481984 == -2.4187

0.1820 - 3.1850 * cos(theta2) == -2.4187

-3.1850 * cos(theta2) == -2.6007

cos(theta2) == 0.8165

theta2 = 0.6155

% Select (1,1) from LHS and RHS

cos(theta1)*cos(theta2) == (cos(theta3)*((1213*cos(8646289787802775/9007199254740992))/(5000*(sin(8646289787802775/9007199254740992)^2 + cos(8646289787802775/9007199254740992)^2)) - (1681*sin(8646289787802775/9007199254740992))/(5000*(sin(8646289787802775/9007199254740992)^2 + cos(8646289787802775/9007199254740992)^2))))/(cos(theta3)^2 + sin(theta3)^2) - (sin(theta3)*((1681*cos(8646289787802775/9007199254740992))/(5000*(sin(8646289787802775/9007199254740992)^2 + cos(8646289787802775/9007199254740992)^2)) + (1213*sin(8646289787802775/9007199254740992))/(5000*(sin(8646289787802775/9007199254740992)^2 + cos(8646289787802775/9007199254740992)^2))))/(cos(theta3)^2 + sin(theta3)^2)

0.3385 == (cos(theta3)*(0.1391 - (1681*0.8192)/(5000))) - (sin(theta3)*((1681*cos(8646289787802775/9007199254740992))/5000 + (1213*0.8192)/5000))

0.3385 == cos(theta3) * -0.1363 - sin(theta3) * 0.3915

% Select (2,1) from LHS and RHS

0.8165*-0.9100 == (sin(theta3)*((7379*cos(8646289787802775/9007199254740992))/(10000*(sin(8646289787802775/9007199254740992)^2 + cos(8646289787802775/9007199254740992)^2)) + (213*sin(8646289787802775/9007199254740992))/(400*(sin(8646289787802775/9007199254740992)^2 + cos(8646289787802775/9007199254740992)^2))))/(cos(theta3)^2 + sin(theta3)^2) - (cos(theta3)*((213*cos(8646289787802775/9007199254740992))/(400*(sin(8646289787802775/9007199254740992)^2 + cos(8646289787802775/9007199254740992)^2)) - (7379*sin(8646289787802775/9007199254740992))/(10000*(sin(8646289787802775/9007199254740992)^2 + cos(8646289787802775/9007199254740992)^2))))/(cos(theta3)^2 + sin(theta3)^2)

-0.7430 == sin(theta3) * 0.8594 - cos(theta3)* -0.2991

% System of equations from (1,1) and (2,1)

0.3385 == cos(theta3) * -0.1363 - sin(theta3) * 0.3915
-0.7430 == sin(theta3) * 0.8594 - cos(theta3)* -0.2991

% Solve for cos(theta3)

0.3385 == cos(theta3) * -0.1363 - sin(theta3) * 0.3915

0.3385 == cos(theta3) * -0.1363 - sin(theta3) * 0.3915

0.3385 + sin(theta3) * 0.3915 == cos(theta3) * -0.1363

-2.4835 + -2.8723 * sin(theta3) == cos(theta3)

% Solve for sin(theta3)

-0.7430 == sin(theta3) * 0.8594 - (-2.4835 + -2.8723 * sin(theta3)) * -0.2991

-0.7430 == sin(theta3) * 0.8594 + (-2.4835 - 2.8723 * sin(theta3)) * 0.2991

-0.7430 == 0.8594 * sin(theta3) + -0.7428 - 0.8591 * sin(theta3)

-0.7430 == 0.8594 * sin(theta3) - 0.7428 - 0.8591 * sin(theta3)

-2.0000e-04 == 0.8594 * sin(theta3) - 0.8591 * sin(theta3)

-2.0000e-04 == 0.0003 * sin(theta3)

-2.0000e-04 / 0.0003 == sin(theta3)

-0.6667 == sin(theta3)

% Substitute into the first equation in the system

-2.4835 + -2.8723 * -0.6667 == cos(theta3)

-0.5685 == cos(theta3)

% Now we have both sin and cos of theta3

sin(theta3) == -0.6667
cos(theta3) == -0.5685

% Solve for theta3

theta3 == atan2(-0.5685, -0.6667)

theta3 == -2.4355