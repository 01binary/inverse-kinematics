% Extract pitch and yaw angles from unit vector
function S = eu(unitVector)
  x = unitVector(1,1);
  y = unitVector(2,1);
  z = unitVector(3,1);

  syms pitch yaw real

  [yaw, pitch] = solve(...
    [
      yaw == atan2(y / cos(pitch), x / cos(pitch)),...
      pitch == atan2(z, sqrt(x^2 + y^2))
    ],...
    [
      yaw,...
      pitch
    ]...
  );

  S.yaw = vpa(yaw,4);
  S.pitch = vpa(pitch,4);
end
