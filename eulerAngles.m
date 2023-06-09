% Extract pitch and yaw angles from unit vector
function S = eulerAngles(unitVector)
  x = unitVector(1,1);
  y = unitVector(2,1);
  z = unitVector(3,1);

  syms pitch yaw

  S = solve(...
    [
      yaw == atan2(y / cos(pitch), x / cos(pitch)),...
      pitch == atan2(z, sqrt(x^2 + y^2))
    ],...
    [
      pitch,...
      yaw
    ]...
  );
end
