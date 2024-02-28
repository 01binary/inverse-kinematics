function frame = translate(x, y, z)
  arguments
    x sym
    y sym
    z sym
  end
  frame = [
    1, 0, 0, x;
    0, 1, 0, y;
    0, 0, 1, z;
    0, 0, 0, 1;
  ];
end
