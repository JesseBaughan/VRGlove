%% Housekeeping
 
addpath('ximu_matlab_library');	% include x-IMU MATLAB library
addpath('quaternion_library');	% include quatenrion library
% close all;                     	% close all figures
% clear;                         	% clear all variables
% clc;                          	% clear the command terminal

%% Import data
% xIMUdata = xIMUdataClass('LoggedData/LoggedData');
xIMUdata = csvread('IMU_accelData.csv');

samplePeriod = 1/256;

% gyr = [xIMUdata.CalInertialAndMagneticData.Gyroscope.X...
%        xIMUdata.CalInertialAndMagneticData.Gyroscope.Y...
%        xIMUdata.CalInertialAndMagneticData.Gyroscope.Z];        % gyroscope
% acc = [xIMUdata.CalInertialAndMagneticData.Accelerometer.X...
%        xIMUdata.CalInertialAndMagneticData.Accelerometer.Y...
%        xIMUdata.CalInertialAndMagneticData.Accelerometer.Z];
   

   
%% Do stuff
%Convert quarternions to rotation matrix

L = 11617;
first = true;
linAccel_prev = zeros(L,3);
linAccel_curr = zeros(L,3);
linVel_prev = zeros(L,3);
linVel_curr= zeros(L,3);
linPos_prev = zeros(L,3);
linPos_curr = zeros(L,3);
tiltCompAccel = zeros(L,3);

samplePeriod = 1/256;

qw = 1;
qx = 0.000027602;
qy = 0.000053161;
qz = 0.0000042611;
R = zeros(3,3,length(gyr));     % rotation matrix describing sensor relative to Earth
ahrs = MahonyAHRS('SamplePeriod', samplePeriod, 'Kp', 1);
for j = 1:1:3
    linAccel_prev(j) = 0;
    linVel_prev(j) = 0;
    linPos_prev(j) = 0;
end

for i = 1:1:10000
    ahrs.UpdateIMU(gyr(i,:) * (pi/180), acc(i,:));	% gyroscope units must be radians
    qw = ahrs.Quaternion(1,1);
    qx = ahrs.Quaternion(1,2);
    qy = ahrs.Quaternion(1,3);
    qz = ahrs.Quaternion(1,4);
    
    [tiltCompAccel(i,:),R(:,:,i)] = CalcTiltCompAcc(qw, qx, qy, qz, acc(i,1), acc(i,2), acc(i,3));
    linAccel_curr(i,:) = CalcLinearAccel_EarthFrame(tiltCompAccel(i,:));
    linAccel_curr(1,:) = [-0.281552802006605,0.162308091998503,0.167691867060753];
%     linAccel_curr(2,:) = [0.0247056031558763, -0.0243002467571364, -0.258665040263768];


end

for i=2:1:10000
    linVel_curr(i,:) = CalcLinearVel(linVel_prev(i-1,:), linAccel_curr(i,:));
        
    %Replace old values with latest
    for k = 1:1:3
        linAccel_prev(i,k) = linAccel_curr(i,k);
    end
    
    %linVelHP = [0.0266719973969310, -0.0353662815543542, -0.00504341145068237];
    
    linPos_curr(i,:) = CalcLinearPos(linPos_prev(i-1,:), linVel_curr(i,:));
    
    %Replace old values with latest
    for m = 1:1:3
        linVel_prev(i,m) = linVel_curr(i,m);
    end
    
    %Replace old values with latest
    for n = 1:1:3
        linPos_prev(i,n) = linPos_curr(i,n);
    end
end
    
SamplePlotFreq = 8;

SixDOFanimation(linPos_curr, R, ...
                'SamplePlotFreq', SamplePlotFreq, 'Trail', 'Off', ...
                'Position', [9 39 1280 720], ...
                'AxisLength', 0.1, 'ShowArrowHead', false, ...
                'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, 'Title', 'Unfiltered',...
                'CreateAVI', false, 'AVIfileNameEnum', false, 'AVIfps', ((1/samplePeriod) / SamplePlotFreq));   

