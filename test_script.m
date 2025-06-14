%% Housekeeping
 
addpath('ximu_matlab_library');	% include x-IMU MATLAB library
addpath('quaternion_library');	% include quatenrion library
close all;                     	% close all figures
clear;                         	% clear all variables
clc;                          	% clear the command terminal
 
%% Import data

xIMUdata = xIMUdataClass('LoggedData/LoggedData');

timestamp = xIMUdata.CalInertialAndMagneticData.Timestamp;
totaltime = timestamp(end) - timestamp(1);         
numPackets = length(timestamp);
samplePeriod =  totaltime / (numPackets - 1);

gyr_0 = [xIMUdata.CalInertialAndMagneticData.Gyroscope.X...
       xIMUdata.CalInertialAndMagneticData.Gyroscope.Y...
       xIMUdata.CalInertialAndMagneticData.Gyroscope.Z];        % gyroscope
acc_0 = [xIMUdata.CalInertialAndMagneticData.Accelerometer.X...
       xIMUdata.CalInertialAndMagneticData.Accelerometer.Y...
       xIMUdata.CalInertialAndMagneticData.Accelerometer.Z];	% accelerometer

mag_0 =  [xIMUdata.CalInertialAndMagneticData.Magnetometer.X...
       xIMUdata.CalInertialAndMagneticData.Magnetometer.Y...
       xIMUdata.CalInertialAndMagneticData.Magnetometer.Z];	% magnetometer


% change unit
gyr = gyr_0 * 180 / pi;
acc = acc_0 / 9.81;
mag = mag_0 / 100;
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

%% Estimate initial orientation from accelerometer

acc0 = acc(1,:);
ax = acc0(1); ay = acc0(2); az = acc0(3);

roll0  = atan2(ay, az);
pitch0 = atan2(-ax, sqrt(ay^2 + az^2));
yaw0   = 0;

q0 = rotMat2quatern(euler2rotMat(roll0, pitch0, yaw0)); % euler to quatern

%% Process data through AHRS algorithm with initial orientation

R = zeros(3,3,length(gyr));     % rotation matrix describing sensor relative to Earth
%ahrs = MahonyAHRS('SamplePeriod', samplePeriod, 'Kp', 1);
ahrs = MahonyAHRS('SamplePeriod', samplePeriod, 'Kp', 1, 'Quaternion', q0);  % initialization with estimated pose
for i = 1:length(gyr)
    ahrs.Update(gyr(i,:) * (pi/180), acc(i,:), mag(i,:));	% gyroscope units must be radians
    R(:,:,i) = quatern2rotMat(ahrs.Quaternion)';    % transpose because ahrs provides Earth relative to sensor
end

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
% Calculate gravity in Earth frame using rotation matrix (sensor relative to Earth)
gravityEarth = [0; 0; 1];

gravitySensor = zeros(size(acc));
linAcc_sensor = zeros(size(acc));
linAcc = zeros(size(acc));

for i = 1:size(acc,1)
    % transfer gravity from Earth frame to Sensor frame
    gravitySensor_i = R(:,:,i)' * gravityEarth;  % column vector
    gravitySensor(i,:) = gravitySensor_i';

    % calculate linear acceleration（sensor frame）
    linAcc_sensor(i,:) = acc(i,:) - gravitySensor(i,:);

    % transfer to Earth frame
    %linAcc(i,:) = (R(:,:,i) * linAcc_sensor(i,:)')';
end

% convert to m/s²
linAcc = linAcc_sensor * 9.81;
%linAcc = tcAcc - [zeros(length(tcAcc), 1), zeros(length(tcAcc), 1), ones(length(tcAcc), 1)];
%linAcc = linAcc * 9.81;     % convert from 'g' to m/s/s

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

order = 1
filtCutOff = 0.2;
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

order = 1;
filtCutOff = 0.2;
[b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high');
linPosHP = filtfilt(b, a, linPos);

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
SamplePlotFreq = 1;

SixDOFanimation(linPosHP, R, ...
                'SamplePlotFreq', SamplePlotFreq, 'Trail', 'Off', ...
                'Position', [9 39 1280 720], ...
                'AxisLength', 0.1, 'ShowArrowHead', false, ...
                'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, 'Title', 'Unfiltered',...
                'CreateAVI', false, 'AVIfileNameEnum', false, 'AVIfps', ((1/samplePeriod) / SamplePlotFreq)); 
 
%% End of script
% Convert rotation matrix R to Euler angles (roll, pitch, yaw)
eulerAngles = zeros(length(R), 3);  % default roll/pitch/yaw

for i = 1:length(R)
    eulerAngles(i,:) = rotMat2euler(R(:,:,i)) * (180/pi);  % transfer to degrees
end

% eulerAngles [roll, pitch, yaw] (deg)
% rotation angle

%eulerAngles = eulerAngles - eulerAngles(end, :);

%linPosHP = linPosHP - linPosHP(end, :);
linPos_mm = 1000 * linPosHP;

% sensor timestamps to ultrasound timestamps

secOfDay = round(mod(timestamp, 86400) + 28800, 3);

% table
T_out = table(secOfDay,...
              linPos_mm(:,1), linPos_mm(:,2), linPos_mm(:,3), ...
              eulerAngles(:,1), eulerAngles(:,2), eulerAngles(:,3), ...
              'VariableNames', {'Timestamp (s)', 'X (mm)', 'Y (mm)', 'Z (mm)', 'Roll (deg)', 'Pitch (deg)', 'Yaw (deg)'});

% output CSV
%writetable(T_out, 'output_position_and_orientation.csv');
%writetable(T_out, 'linear_position_test_data1_3.csv');
writetable(T_out, 'linear_position_0506_3_new.csv');

% linPosHP_mm = 1000.* linPosHP;
% data_to_save = [linPosHP_mm, timestamp]; %% time stamp included
% writematrix(["X (mm)", "Y (mm)", "Z (mm)", "Roll_deg", "Pitch_deg", "Yaw_deg", "Timestamp (s)"; data_to_save], 'linear_position_test_data1_1.csv');