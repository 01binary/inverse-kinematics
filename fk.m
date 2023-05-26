% Calculate forward kinematics given joint angles
function Effector = fk(theta1, theta2, theta3)
  arguments
    theta1  sym
    theta2  sym
    theta3  sym
  end
  %             length     offset     twist            angle
  Base =      dh(a = 0,    d = 1,     alpha = pi / 2,  theta = theta1)
  Offset =    dh(a = -0.2, d = 0,     alpha = 0,       theta = 0)
  Shoulder =  dh(a = 3.5,  d = 0,     alpha = 0,       theta = theta2)
  Elbow =     dh(a = 3.5,  d = 0,     alpha = 0,       theta = theta3)
  Wrist =     dh(a = 2.5,  d = -0.18, alpha = 0,       theta = 0.959931)

  Effector = Base * Offset * Shoulder * Elbow * Wrist
end