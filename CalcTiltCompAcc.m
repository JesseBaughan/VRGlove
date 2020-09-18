function [t_a, R] = CalcTiltCompAcc(qw, qx, qy, qz, ax, ay, az)
%   float qx = *(getQ() + 1);
%   float qy = *(getQ() + 2);
%   float qz = *(getQ() + 3);
%   Mock values for testing

   sqw = qw*qw;
   sqx = qx*qx;
   sqy = qy*qy;
   sqz = qz*qz;

%   // invs (inverse square length) is only required if quaternion is not already normalised
%   //OUr quarternions are already normalised
%   //float invs = 1 / (sqx + sqy + sqz + sqw);
   invs = 1; %//delete this and all refs to it later on

%   //Calculate the rotation matrix from quarternions
   m00 = ( sqx - sqy - sqz + sqw)*invs ; %// since sqw + sqx + sqy + sqz =1/invs*invs
   m11 = (-sqx + sqy - sqz + sqw)*invs ;
   m22 = (-sqx - sqy + sqz + sqw)*invs ;
  
   tmp1 = qx*qy;
   tmp2 = qz*qw;
   m10 = 2.0 * (tmp1 + tmp2)*invs ;
   m01 = 2.0 * (tmp1 - tmp2)*invs ;
  
   tmp1 = qx*qz;
   tmp2 = qy*qw;
   m20 = 2.0 * (tmp1 - tmp2)*invs ;
   m02 = 2.0 * (tmp1 + tmp2)*invs ;
  tmp1 = qy*qz;
  tmp2 = qx*qw;
   m21 = 2.0 * (tmp1 + tmp2)*invs ;
   m12 = 2.0 * (tmp1 - tmp2)*invs ;
   
   R = [m00, m01, m02; m10, m11, m12; m20, m21, m22];

%   //Multiply rotation matrix by accell values to get tilt-compensated accel vals
  r00 = (m00 * ax) + (m01 * ay) + (m02 * az);
  r01 = (m10 * ax) + (m11 * ay) + (m12 * az);
  r02 = (m20 * ax) + (m21 * ay) + (m22 * az);

  t_a = zeros(1, 3);
  t_a(1) = r00; 
  t_a(2) = r01;
  t_a(3) = r02;
  
end

