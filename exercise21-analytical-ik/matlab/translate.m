function frame = translate(x, y, z)
  frame = [
    1 0 0 x;
    0 1 0 y;
    0 0 1 z;
    0 0 0 1
  ];
end