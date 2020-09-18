function  linVel_curr = CalcLinearVel(linVel_prev, linAccel_curr)
%   //Perform first integration to go from accel to vel
  linVel_curr = zeros(1,3);
  for i = 1:1:3
    linVel_curr(1,i) = linVel_prev(1,i) + linAccel_curr(1,i) * (1/256);
  end
  
end

