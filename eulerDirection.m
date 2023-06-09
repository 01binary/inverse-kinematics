% Calculate a 3D unit vector from euler angles
function S = eulerDirection(pitch, yaw)
  S = [
    cos(yaw) * cos(pitch);
    cos(pitch) * sin(yaw);
    sin(pitch)
  ];
end
