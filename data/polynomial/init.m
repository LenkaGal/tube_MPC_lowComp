%% FLEXY2 
% 2019-04-09

%% Prerequisities
addpath('RealTime_Pacer');    % if not included in the PATH
slCharacterEncoding('UTF-8'); % helps avoid warnings

%% Initialization and communication PORT identification
f = Flexy2("COM8");

%% Double calibration
f.calibrate;

%% Close CLI connection to enable Simulink communication
f.close

%% Initialize communication
% Sampling time
Ts = 0.01;
% Calibration
calibrate = 1;
% Port = 'COM4';
Port = f.ComPort;




