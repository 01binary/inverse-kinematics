% Let's try basic reverse calculation given a perfect answer going forward

Effector = fk(-1.143311, 0.6155, -2.5211)

Effector = [
    0.2426    0.3362   -0.9100    1.3954
   -0.5325   -0.7379   -0.4146   -2.6287
   -0.8109    0.5852    0.0000   -2.3121
         0         0         0    1.0000
];

% OK, now apply the solving method with variable matrices

Base =      dh(a = 0,    d = 1,     alpha = pi / 2,  theta = theta1)
Offset =    dh(a = -0.2, d = 0,     alpha = 0,       theta = 0)
Shoulder =  dh(a = 3.5,  d = 0,     alpha = 0,       theta = theta2)
Elbow =     dh(a = 3.5,  d = 0,     alpha = 0,       theta = theta3)
Wrist =     dh(a = 2.5,  d = -0.18, alpha = 0,       theta = 0.959931)

% Solve the equation

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

% Select (1,4) from LHS and RHS

cos(theta2) == 0.8166

% Solve for theta2 (Shoulder)

theta2 = 0.6153

% Select (3,1) from LHS and RHS

0.4713 == cos(theta3) * 0.2991 + sin(theta3) * 0.8595

% Select (1,2) from LHS and RHS

-0.2393 == cos(theta3) * 0.3915 + sin(theta3) * -0.1363

% We got a system of equations with theta3 as the unknown
% Solve for sin(theta3) in terms of cos(theta3)

0.4713 == cos(theta3) * 0.2991 + sin(theta3) * 0.8595

0.4713 - cos(theta3) * 0.2991 == sin(theta3) * 0.8595

(0.4713 - cos(theta3) * 0.2991) / 0.8595 == sin(theta3)

0.5483 - cos(theta3) * 0.3480 == sin(theta3)

% Solve for cos(theta3) by subbing into the other equation
% Don't take the cosine yet, we are setting up for atan2

-0.2393 == cos(theta3) * 0.3915 + sin(theta3) * -0.1363

-0.2393 == cos(theta3) * 0.3915 + 0.5483 - cos(theta3) * 0.3480 * -0.1363

-0.2393 == cos(theta3) * 0.3915 + 0.5483 - cos(theta3) * -0.0474

-0.7876 == 0.3915 * cos(theta3) + 0.0474 * cos(theta3)

-0.7876 == 0.4389 * cos(theta3)

-1.7945 == cos(theta3)

% Solve for sin(theta3) by subbing back into first equation
% Don't take the sine yet, we are setting up for atan2

0.4713 == -1.7945 * 0.2991 + sin(theta3) * 0.8595

0.4713 == sin(theta3) * 0.8595 - 0.5367

1.008 == sin(theta3) * 0.8595

1.1728 == sin(theta3)

% Now that we know both sin and cos of theta3, we can take atan2

cos(theta3) == -1.7945
sin(theta3) == 1.1728

theta3 == atan2(-1.7945, 1.1728)

theta3 = -0.9919

% All joint variables found...

theta1 = -1.1433
theta2 = 0.6153
theta3 = -0.9919

% Compare with what we set in Blender

theta1b = -1.1433
theta2b = 0.6155
theta3b = -2.5211

% theta3 came out wrong
% Is it my math work that made it wrong, or not following the algorithm?

