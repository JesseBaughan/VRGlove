function linAcc = CalcLinearAccel_EarthFrame(t_a)
%   //Sub tract earth frame (minus gravity from Z axis)

  linAcc = zeros(1,3);
  
%   //Convert to m/s/s
  linAcc(1,1) = t_a(1,1) * 9.81;
  linAcc(1,2) = t_a(1,2) * 9.81;
  linAcc(1,3) = (t_a(1,3)-1) * 9.81;
  
end

