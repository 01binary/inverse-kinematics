% Calculate joint angles given end effector
% TODO how to loop and what's the matrix type?
function IK = ik(frames, ee)
arguments
  frames  double4x4[]
  ee      double4x4[]
end

  lhs = eye(4,4)
  rhs = ee

  % for each frame in frames array, which are 4x4 matrices
  % multiply lhs by frame
  % lhs = lhs * frame

  

  foreach (frame in frames)
    lhs = lhs * frame;

  IK = solve(lhs == rhs);
end
