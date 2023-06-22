% Calculate joint angles given end effector
function IK = ik(frames, ee)
  lhs = eye(4,4)
  rhs = ee

  for i = 1:length(frames)
    lhs = lhs * frames(i);
  end

  IK = solve(lhs == rhs);
end
