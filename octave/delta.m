% Delta robot inverse kinematics

% Arm | Shoulder Offset          | Wrist Offset
%-----|--------------------------|-------------------------
% 1   | 0.798967, 0.0, 0.0       | 0.399483, 0.0, 0.0
% 2   | -0.399483, -0.69194, 0.0 | -0.199742, -0.34597, 0.0
% 3   | -0.399483, 0.69194, 0.0  | -0.199742, 0.34597, 0.0

% Example usage to calculate angles for arm 1
%   Choose shoulder and wrist offsets from the table, then plug in target position
%
%   [baseAngle, shoulderAngle, elbowAngle] = delta(
%     struct(
%       "shoulder", [0.798967; 0.0; 0.0],
%       "wrist", [0.399483; 0.0; 0.0],
%       "target", [-0.299456; 0.0; -1.81914]
%     )
%   )

function [baseAngle, shoulderAngle, elbowAngle] = delta(parameters)
  % Constants

  upperArmLength = 1.0678;
  forearmLength = 1.28925;

  % Parameters

  shoulderOffsetX = parameters.shoulder(1,1);
  shoulderOffsetY = parameters.shoulder(2,1);
  shoulderOffsetZ = parameters.shoulder(3,1);

  wristOffsetX = parameters.wrist(1,1);
  wristOffsetY = parameters.wrist(2,1);
  wristOffsetZ = parameters.wrist(3,1);

  targetX = parameters.target(1,1);
  targetY = parameters.target(2,1);
  targetZ = parameters.target(3,1);

  % Step 1 - Calculate wrist position in world space
  %   backtrack from target to wrist using wrist offsets

  wristX = targetX + wristOffsetX;
  wristY = targetY + wristOffsetY;
  wristZ = targetZ + wristOffsetZ;

  % Step 2 - Calculate angle from shoulder to wrist on XY plane
  %   tan(angle) = opposite / adjacent
  %   opposite is shoulder to wrist distance on Y axis
  %   adjacent is shoulder to wrist distance on X axis

  baseAngle = atan2(shoulderOffsetY - wristY, shoulderOffsetX - wristX);

  % Step 3 - Calculate the distance between shoulder and wrist
  %   This is used as the hypotenuse to calculate shoulder and elbow angles

  shoulderToWristSq = (
    (wristX - shoulderOffsetX)^2 +
    (wristY - shoulderOffsetY)^2 +
    (wristZ - shoulderOffsetZ)^2
  );

  shoulderToWrist = sqrt(shoulderToWristSq);

  % Step 4 - Calculate angle from shoulder to wrist on arm plane
  %   This is used as a "baseline" for the shoulder angle
  %   sin(angle) = opposite / hypotenuse
  %   angle = asin(opposite / hypotenuse)

  shoulderWristAngle = asin(
    (wristZ - shoulderOffsetZ) / shoulderToWrist
  ) + pi/2;

  % Step 5 - Calculate shoulder angle
  %   Invoke law of cosines
  %   a^2 == b^2 + c^2 - 2bc * cos(alpha)
  %   cos(alpha) == (a^2 - b^2 - c^2) / -2bc
  %   c == upperArmLength
  %   a == forearmLength
  %   b == distance between shoulder and wrist
  %   subtract result from shoulderWristAngle because
  %   we want the arm elbow to bend outward from the baseline

  shoulderAngle = shoulderWristAngle - acos(
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
  );

  % Step 6 - Calculate elbow angle
  %   Invoke law of cosines
  %   b^2 == a^2 + c^2 - 2ac * cos(beta)
  %   cos(beta) = (b^2 - a^2 - c^2) / -2ac
  %   all variables same as before:
  %   c == upperArmLength
  %   a == forearmLength
  %   b == distance between shoulder and wrist
  %   subtract result from pi (180deg) since we want the angle
  %   between the upper arm and the forearm

  elbowAngle = (pi - acos(
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
  ));
end
