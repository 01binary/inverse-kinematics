% Constants

upperArmLength = 1.0678;
forearmLength = 1.28925;

% Parameters for each arm

% Arm | Shoulder Offset          | Wrist Offset
%-----|--------------------------|-------------------------
% 1   | 0.798967, 0.0, 0.0       | 0.399483, 0.0, 0.0
% 2   | -0.399483, -0.69194, 0.0 | -0.199742, -0.34597, 0.0
% 3   | -0.399483, 0.69194, 0.0  | -0.199742, 0.34597, 0.0

shoulderOffsetX = -0.399483;
shoulderOffsetY = 0.69194;
shoulderOffsetZ = 0;

wristOffsetX = -0.199742;
wristOffsetY = 0.34597;
wristOffsetZ = 0;

% Input - target for inverse kinematics

targetX = -0.299456;
targetY = 0;
targetZ = -1.81914;

% Step 1 - Calculate wrist position in world space
% Backtrack from target to wrist using wrist offsets

wristX = targetX + wristOffsetX
wristY = targetY + wristOffsetY
wristZ = targetZ + wristOffsetZ

% Step 2 - Calculate rotation angle from shoulder to wrist
% tan(angle) = opposite / adjacent
% opposite is shoulder to wrist distance on Y axis
% adjacent is shoulder to wrist distance on X axis

baseAngle = atan2(shoulderOffsetY - wristY, shoulderOffsetX - wristX)

% Step 3 - Calculate the distance between shoulder and wrist

shoulderToWristSq = (
  (wristX - shoulderOffsetX)^2 +
  (wristY - shoulderOffsetY)^2 +
  (wristZ - shoulderOffsetZ)^2
)

shoulderToWrist = sqrt(shoulderToWristSq)

% Shoulder to wrist angle
% sin(angle) = opposite / hypotenuse
% angle = asin(opposite / hypotenuse)

shoulderWristAngle = asin(
  (wristZ - shoulderOffsetZ) / shoulderToWrist
) + pi/2

% Step 4 - Calculate shoulder angle
% Invoke law of cosines
% a^2 == b^2 + c^2 - 2bc * cos(alpha)
% cos(alpha) == (a^2 - b^2 - c^2) / -2bc
% c == upperArmLength
% a == forearmLength
% b == distance between shoulder and wrist

shoulderAngle = -(acos(
  (
    forearmLength ^ 2 -
    shoulderToWristSq -
    upperArmLength ^ 2
  )
  /
  (
    - 2 *
    shoulderToWrist *
    upperArmLength
  )
) - shoulderWristAngle)

% Step 5 - Calculate elbow angle
% Invoke law of cosines
% b^2 == a^2 + c^2 - 2ac * cos(beta)
% cos(beta) = (b^2 - a^2 - c^2) / -2ac
% all variables same as before:
% c == upperArmLength
% a == forearmLength
% b == distance between shoulder and wrist

elbowAngle = pi - acos(
  (
    shoulderToWristSq -
    forearmLength^2 -
    upperArmLength^2
  )
  /
  (
    -2 *
    forearmLength *
    upperArmLength
  )
)