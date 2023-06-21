% Calculate forward kinematics given joint angles
% TODO make this loop
function Effector = fk(theta1, theta2, theta3)
  arguments
    theta1  sym
    theta2  sym
    theta3  sym
  end
  %           length            offset        twist            angle
  Base =      dh(a = 0,         d = 0.11518,  alpha = pi / 2,  theta = theta1);
  Offset =    dh(a = -0.013,    d = 0,        alpha = 0,       theta = 0);
  Shoulder =  dh(a = 0.4173,    d = 0,        alpha = 0,       theta = theta2);
  Elbow =     dh(a = 0.48059,   d = 0,        alpha = 0,       theta = theta3);
  Wrist =     dh(a = 0.008,     d = 0,        alpha = 0,       theta = pi / 4);
  Drumstick = dh(a = 0.295,     d = -0.023,   alpha = 0,       theta = -pi / 4 + 0.959931);

  Effector = Base * Offset * Shoulder * Elbow * Wrist * Drumstick;
end
