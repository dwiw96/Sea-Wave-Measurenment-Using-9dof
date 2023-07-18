close all;                     	% close all figures
clear;                         	% clear all variables
clc;                          	% clear the command terminal

%% Housekeeping
 
addpath('github.com\dwiw96\Sea-Wave-Measurenment-Using-9dof\quaternion_library');	% include quatenrion library
addpath('github.com\dwiw96\Sea-Wave-Measurenment-Using-9dof\GyroscopeIntegration');    % include Gyroscope integration library
addpath('github.com\dwiw96\Sea-Wave-Measurenment-Using-9dof\AccelerometerMagnetometer');    % include Accelerometer and Magnetometer Integration library

%% Import data

[~, ~, raw] = xlsread('D:\Uyung\TA\Tsunami\Program\testing\loggedData_3.xlsx','sheet3','A1:Q1084');

% Allocate imported array to column variable names
% Create output variable
data = reshape([raw{:}],size(raw));
linAccRaw = data(:,2:4);
acc = data(:,5:7);
gyro = data(:,8:10);
mag = data(:,11:13);
q = data(:,14:17);
time = data(:,1);

% convert degree to radian
% gyr = deg2rad(gyro);

samplePeriod = 1/16;

% Plot
figure('NumberTitle', 'off', 'Name', 'Accelerometer');
hold on;
plot(acc(:,1), 'r');
plot(acc(:,2), 'g');
plot(acc(:,3), 'b');
xlabel('sample');
ylabel('g');
title('Accelerometer');
legend('X', 'Y', 'Z');

figure('NumberTitle', 'off', 'Name', 'Gyroscope');
hold on;
plot(gyro(:,1), 'r');
plot(gyro(:,2), 'g');
plot(gyro(:,3), 'b');
xlabel('sample');
ylabel('dps');
title('Gyroscope');
legend('X', 'Y', 'Z');

figure('NumberTitle', 'off', 'Name', 'Magnetometer');
hold on;
plot(mag(:,1), 'r');
plot(mag(:,2), 'g');
plot(mag(:,3), 'b');
xlabel('sample');
ylabel('Gauss');
title('Magnetometer');
legend('X', 'Y', 'Z');

%% Process data through AHRS algorithm (calcualte orientation)
% See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/

R = zeros(3,3,length(gyro));     % rotation matrix describing sensor relative to Earth

AHRS = MadgwickAHRS('SamplePeriod', samplePeriod, 'Beta', 0.1);

quaternion = zeros(length(time),4);
for t = 1:length(time)
    AHRS.Update(gyro(t,:) * (pi/180), acc(t,:), mag(t,:));	% gyroscope units must be radians
    quaternion(t, :) = AHRS.Quaternion;
    R(:,:,t) = quatern2rotMat(AHRS.Quaternion)';
end

% Quaternion to rotation matrix
%R = quatern2rotMat(quaternion);

%% Calculate 'tilt-compensated' accelerometer

tcAcc = zeros(size(acc));  % accelerometer in Earth frame

for i = 1:length(acc)
    tcAcc(i,:) = R(:,:,i) * acc(i,:)';
end

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

linAcc = tcAcc - [zeros(length(tcAcc), 1), zeros(length(tcAcc), 1), ones(length(tcAcc), 1)];
linAcc = linAcc * 9.81;     % convert from 'g' to m/s/s
linAcc_cuplik = linAcc(750:800,:);

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

%% High-pass filter linear Acceleration to remove drift
order = 4;
filtCutOff = 0.1;
[b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high');
linAccHP = filtfilt(b, a, linAcc);

%% Calculate linear velocity (integrate acceleartion)
linVel = zeros(size(linAccHP));

for i = 2:length(linAccHP)
    linVel(i,:) = linVel(i-1,:) + linAccHP(i,:) * samplePeriod;
end

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

order = 4;
filtCutOff = 0.1;
[b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high');
linVelHP = filtfilt(b, a, linVel);

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

order = 4;
filtCutOff = 0.1;
[b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high');
linPosHP = filtfilt(b, a, linPos);
linPosHP = linPosHP*100;

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

SamplePlotFreq = 8;

SixDOFanimation(linPosHP, R, ...
                'SamplePlotFreq', SamplePlotFreq, 'Trail', 'Off', ...
                'Position', [9 39 1280 720], ...
                'AxisLength', 0.1, 'ShowArrowHead', false, ...
                'Xlabel', 'X (cm)', 'Ylabel', 'Y (cm)', 'Zlabel', 'Z (cm)', 'ShowLegend', false, 'Title', 'Unfiltered',...
                'CreateAVI', false, 'AVIfileNameEnum', false, 'AVIfps', ((1/samplePeriod) / SamplePlotFreq));            
 
%% End of script
