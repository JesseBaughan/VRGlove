%% Housekeeping
 
addpath('ximu_matlab_library');	% include x-IMU MATLAB library
addpath('quaternion_library');	% include quatenrion library
close all;                     	% close all figures
clear;                         	% clear all variables
clc;                          	% clear the command terminal
 
%% Import data

% xIMUdata = xIMUdataClass('LoggedData/LoggedData');

% samplePeriod = 0.002;

% gyr = [xIMUdata.CalInertialAndMagneticData.Gyroscope.X...
%        xIMUdata.CalInertialAndMagneticData.Gyroscope.Y...
%        xIMUdata.CalInertialAndMagneticData.Gyroscope.Z];        % gyroscope
% acc = [xIMUdata.CalInertialAndMagneticData.Accelerometer.X...
%        xIMUdata.CalInertialAndMagneticData.Accelerometer.Y...
%        xIMUdata.CalInertialAndMagneticData.Accelerometer.Z];	% accelerometer

xIMUdata = csvread('IMU_accelData.csv');
      acc = [xIMUdata(:,1)...
       xIMUdata(:,2)...
       xIMUdata(:,3)];	
   
      gyr = [xIMUdata(:,4)...
       xIMUdata(:,5)...
       xIMUdata(:,6)];    
   
   qw = xIMUdata(:,7);
   qx = xIMUdata(:,8);
   qy = xIMUdata(:,9);
   qz = xIMUdata(:,10);
   
   quarternion = [qw, qx, qy, qz];
   
%    acc = acc(1200:end,:);
%    gyr = gyr(1200:end,:);
%    quarternion = quarternion(1200:end,:);
%    
%    acc = acc(1:1800,:);
%    gyr = gyr(1:1800,:);
%    quarternion = quarternion(1:1800,:);

% samplePeriod = xIMUdata(:,7);
samplePeriod = 1/256;
t = 1:length(acc(:,1));
  
% Plot
figure('NumberTitle', 'off', 'Name', 'Gyroscope');
hold on;
plot(gyr(:,1), 'r');
plot(gyr(:,2), 'g');
plot(gyr(:,3), 'b');
xlabel('sample');
ylabel('dps');
title('Gyroscope');
legend('X', 'Y', 'Z');

figure('NumberTitle', 'off', 'Name', 'Accelerometer');
hold on;
plot(acc(:,1), 'r');
plot(acc(:,2), 'g');
plot(acc(:,3), 'b');
xlabel('sample');
ylabel('g');
title('Accelerometer');
legend('X', 'Y', 'Z');

% WINDOW_SIZE = 5;
% SUM = zeros(1,3);
% READINGS = zeros(WINDOW_SIZE,3);
% INDEX = 0;
% 
% for i = 1:length(gyr)
%     SUM = SUM - READINGS(INDEX+1,:);      % // Remove the oldest entry from the sum
%     VALUE = acc(i,:);
%     READINGS(INDEX+1,:) = VALUE;          % // Add the newest reading to the window
%     SUM = SUM + VALUE;                % // Add the newest reading to the sum
%     INDEX = mod((INDEX+1),WINDOW_SIZE); % WINDOW_SIZE;   // Increment the index, and wrap to 0 if it exceeds the window size
% 
%     AVERAGED(i,:) = SUM / WINDOW_SIZE;  
% end
% 
% acc = AVERAGED;


% figure('NumberTitle', 'off', 'Name', 'Accelerometer');
% hold on;
% plot(AVERAGED(:,1), 'r');
% plot(AVERAGED(:,2), 'g');
% plot(AVERAGED(:,3), 'b');
% xlabel('sample');
% ylabel('g');
% title('Accelerometer MMA');
% legend('X', 'Y', 'Z');


%%
% HP filter accelerometer data
filtCutOff = 0.05;
[b, a] = butter(2, (2*filtCutOff)/(1/samplePeriod), 'high');
acc = filtfilt(b, a, acc);
% 
% % LP filter accelerometer data
% filtCutOff = 20;
% [b, a] = butter(2, (2*filtCutOff)/(1/samplePeriod), 'low');
% acc = filtfilt(b, a, acc);

% Threshold detection
thresh1 = 0.05;
thresh2 = -thresh1;

% stationary = ((acc < 0.0001) && (acc > -0.0001));

%% Process data through AHRS algorithm (calcualte orientation)
% See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/

R = zeros(3,3,length(gyr));     % rotation matrix describing sensor relative to Earth

ahrs = MahonyAHRS('SamplePeriod', samplePeriod, 'Kp', 1);

for i = 1:length(gyr) 
    ahrs.UpdateIMU(gyr(i,:) * (pi/180), acc(i,:));	% gyroscope units must be radians
%     ahrs.UpdateIMU(gyr(i,:) * (pi/180), acc(i,:));	% gyroscope units must be radians
%     jj = ahrs.Quaternion;
%     R(:,:,i) = quatern2rotMat(ahrs.Quaternion)';    % transpose because ahrs provides Earth relative to sensor
    R(:,:,i) = quatern2rotMat(quarternion(i,:))';    % transpose because ahrs provides Earth relative to sensor
end

%% Calculate 'tilt-compensated' accelerometer

tcAcc = zeros(size(acc));  % accelerometer in Earth frame

for i = 1:length(acc)
    tcAcc(i,:) = R(:,:,i) * acc(i,:)';
end

tAcc = acc;

% Plot
figure('NumberTitle', 'off', 'Name', '''Tilt-Compensated'' accelerometer');
hold on;
plot(tcAcc(:,1), 'r');
plot(tcAcc(:,2), 'g');
plot(tcAcc(:,3), 'b');
xlabel('sample');
ylabel('g');
title('''Tilt-compensated'' accelerometer');
legend('X', 'Y', 'Z');

%% Calculate linear acceleration in Earth frame (subtracting gravity)

linAcc = tcAcc - [zeros(length(tcAcc), 1), zeros(length(tcAcc), 1), ones(length(tcAcc),1)];
linAcc = linAcc * 9.81;     % convert from 'g' to m/s/s

% Plot
figure('NumberTitle', 'off', 'Name', 'Linear Acceleration');
hold on;
plot(linAcc(:,1), 'r');
plot(linAcc(:,2), 'g');
plot(linAcc(:,3), 'b');
xlabel('sample');
ylabel('g');
title('Linear acceleration');
legend('X', 'Y', 'Z');

%% Calculate linear velocity (integrate acceleartion)

linVel = zeros(size(linAcc));

for i = 2:length(linAcc)
    linVel(i,:) = linVel(i-1,:) + linAcc(i,:) * samplePeriod;    
end


% % Compute integral drift during non-stationary periods
% velDrift = zeros(size(linVel));
% stationaryStart = find([0; diff(stationary)] == -1);
% stationaryEnd = find([0; diff(stationary)] == 1);
% for i = 1:numel(stationaryEnd)
%     driftRate = linVel(stationaryEnd(i)-1, :) / (stationaryEnd(i) - stationaryStart(i));
%     enum = 1:(stationaryEnd(i) - stationaryStart(i));
%     drift = [enum'*driftRate(1) enum'*driftRate(2) enum'*driftRate(3)];
%     velDrift(stationaryStart(i):stationaryEnd(i)-1, :) = drift;
% end
% 
% % Remove integral drift
% linVel = linVel - velDrift;

% Plot
figure('NumberTitle', 'off', 'Name', 'Linear Velocity');
hold on;
plot(linVel(:,1), 'r');
plot(linVel(:,2), 'g');
plot(linVel(:,3), 'b');
xlabel('sample');
ylabel('g');
title('Linear velocity');
legend('X', 'Y', 'Z');

%% High-pass filter linear velocity to remove drift
order = 1;
filtCutOff = 0.01;
[b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high');
linVelHP = filtfilt(b, a, linVel);
% linVelHP = linVel;

filtCutOff = 15;
[b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'low');
linVelHP = filtfilt(b, a, linVelHP);

% Plot
figure('NumberTitle', 'off', 'Name', 'High-pass filtered Linear Velocity');
hold on;
plot(linVelHP(:,1), 'r');
plot(linVelHP(:,2), 'g');
plot(linVelHP(:,3), 'b');
xlabel('sample');
ylabel('g');
title('High-pass filtered linear velocity');
legend('X', 'Y', 'Z');

%% Calculate linear position (integrate velocity)
linPos = zeros(size(linVelHP));

for i = 2:length(linVelHP)
%     Force vel to zero when not moving
%     if((linVelHP(i,1) < thresh1 && linVelHP(i,1) > thresh2)) 
%         linVelHP(i,1) = 0;
%     end
%     if (linVelHP(i,2) < thresh1 && linVelHP(i,2) > thresh2)
%         linVelHP(i,2) = 0;
%     end
%     if (linVelHP(i,3) < thresh1 && linVelHP(i,3) > thresh2)
%         linVelHP(i,3) = 0;
%     end
    linPos(i,:) = linPos(i-1,:) + linVelHP(i,:) * samplePeriod;
end

% Plot
figure('NumberTitle', 'off', 'Name', 'Linear Position');
hold on;
plot(linPos(:,1), 'r');
plot(linPos(:,2), 'g');
plot(linPos(:,3), 'b');
xlabel('sample');
ylabel('g');
title('Linear position');
legend('X', 'Y', 'Z');



%% High-pass filter linear position to remove drift
order = 1;
filtCutOff = 1;
[b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high');
linPosHP = filtfilt(b, a, linPos);

% linPosHP(1:650,3) = -0.002;
% linPosHP(:,3) = linPosHP(:,3)*5;
% linPosHP = linPos;

% Plot
figure('NumberTitle', 'off', 'Name', 'High-pass filtered Linear Position');
hold on;
plot(linPosHP(:,1), 'r');
plot(linPosHP(:,2), 'g');
plot(linPosHP(:,3), 'b');
xlabel('sample');
ylabel('g');
title('High-pass filtered linear position');
legend('X', 'Y', 'Z');

%% Play animation

SamplePlotFreq = 3;

% linPosHP = linPosHP * 10; %scale so we can see more easily
SixDOFanimation(linPosHP, R, ...
                'SamplePlotFreq', SamplePlotFreq, 'Trail', 'Off', ...
                'Position', [9 39 1280 720], ...
                'AxisLength', 0.1, 'ShowArrowHead', false, ...
                'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, 'Title', 'Unfiltered',...
                'CreateAVI', false, 'AVIfileNameEnum', false, 'AVIfps', ((1/samplePeriod) / SamplePlotFreq));           
 
%% End of script