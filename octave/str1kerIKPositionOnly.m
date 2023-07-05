% Position-only IK for str1ker

% Goal
Goal = [ 0.7929; 0.3602; 0.7896 ];
goalX = Goal(1,1);
goalY = Goal(2,1);
goalZ = Goal(3,1);

% Constants
baseX = 0;
baseY = 0;
baseZ = 0;
drumstickOffset = -0.023;
drumstickLength = 0.305;
drumstickAngle = 0.959931;
forearmLength = 0.48059;
upperArmLength = 0.4173;
shoulderOffsetX = -0.013;
shoulderOffsetZ = 0.11518;

% Base angle
angleToGoal = atan2(goalY - baseY, goalX - baseX);
distanceToGoalXY = sqrt((goalX - baseX)^2 + (goalY - baseY)^2);
angleWristOffset = abs(asin(drumstickOffset / distanceToGoalXY));
baseAngle = angleToGoal - angleWristOffset;

% Shoulder angle
shoulderX = shoulderOffsetX;
shoulderZ = shoulderOffsetZ;

shoulderToGoalDistance = sqrt((distanceToGoalXY - shoulderX)^2 + (goalZ - shoulderZ)^2);

sin(drumstickAngle) == drumstickOffsetZ / drumstickLength;
drumstickOffsetZ = sin(drumstickAngle) * drumstickLength;

cos(drumstickAngle) == drumstickOffsetX / drumstickLength;
drumstickOffsetX = cos(drumstickAngle) * drumstickLength;

tan(elbowToGoalAngle) == drumstickOffsetZ / (forearmLength + drumstickOffsetX);
elbowToGoalAngle = atan2(drumstickOffsetZ, forearmLength + drumstickOffsetX);

elbowToGoalDistance = sqrt((forearmLength + drumstickOffsetX)^2 + drumstickOffsetZ^2);

innerShoulderAngle = acos(...
  (elbowToGoalDistance^2 - shoulderToGoalDistance^2 - upperArmLength^2) ...
  / ...
  (-2 * shoulderToGoalDistance * upperArmLength) ...
);

shoulderToGoalZ = goalZ - shoulderZ;
shoulderToGoalX = distanceToGoalXY - shoulderX;
shoulderToGoalAngle = atan2(shoulderToGoalZ, shoulderToGoalX);

shoulderAngle = shoulderToGoalAngle + innerShoulderAngle;

shoulderToGoalDistance^2 == elbowToGoalDistance^2 + upperArmLength^2 - 2 * elbowToGoalDistance * upperArmLength * cos(outerElbowAngle);
shoulderToGoalDistance^2 - elbowToGoalDistance^2 - upperArmLength^2 == -2 * elbowToGoalDistance * upperArmLength * cos(outerElbowAngle);
(shoulderToGoalDistance^2 - elbowToGoalDistance^2 - upperArmLength^2) / (-2 * elbowToGoalDistance * upperArmLength) == cos(outerElbowAngle);
outerElbowAngle = acos( ...
  (shoulderToGoalDistance^2 - elbowToGoalDistance^2 - upperArmLength^2) ...
  / ...
  (-2 * elbowToGoalDistance * upperArmLength) ...
);

innerElbowAngle = outerElbowAngle - elbowToGoalAngle;
elbowAngle = -(pi - innerElbowAngle);

% End Effector
endEffectorPitch = shoulderAngle + elbowAngle + drumstickAngle;
endEffectorYaw = baseAngle;

Normal = [
  cos(endEffectorYaw) * cos(endEffectorPitch);
  sin(endEffectorYaw) * cos(endEffectorPitch);
  sin(endEffectorPitch);
];

approachYawAngle = endEffectorYaw - pi / 2;
Approach = [
  cos(approachYawAngle);
  sin(approachYawAngle);
  0
];

Orientation = cross(Approach, Normal);

Pose = [ goalX; goalY; goalZ ];

EE = [ ...
  [ Normal(1,1), Orientation(1,1), Approach(1,1), Pose(1,1) ];
  [ Normal(2,1), Orientation(2,1), Approach(2,1), Pose(2,1) ];
  [ Normal(3,1), Orientation(3,1), Approach(3,1), Pose(3,1) ];
  [ 0, 0, 0, 1 ]
]
