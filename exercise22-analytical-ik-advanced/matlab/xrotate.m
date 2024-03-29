function frame = xrotate(theta)
  arguments
    theta sym
  end
  frame = [
    1, 0, 0, 0;
    0, cos(theta), -sin(theta), 0;
    0, sin(theta), cos(theta), 0;
    0, 0, 0, 1;
  ];
end
