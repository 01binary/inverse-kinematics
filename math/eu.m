% Extract pitch and yaw angles from unit vector
function S = eu2(unitVector)
  x = unitVector(1,1);
  y = unitVector(2,1);
  z = unitVector(3,1);

  pitch = atan2(z, sqrt(x^2 + y^2));
  yaw = atan2(y / cos(pitch), x / cos(pitch));

  S.pitch = pitch;
  S.yaw = yaw;
end
