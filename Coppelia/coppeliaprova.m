% Functions that link Coppelia and Matlab.
clear; close all; clc
addpath('libCoppelia/');
% Connect to CoppeliaSim remote API server
vrep = remApi('remoteApi');
vrep.simxFinish(-1);  % Close all current connections
clientID = vrep.simxStart('127.0.0.1', 19000, true, true, 5000, 5);
% Check connection status
if clientID < 0
    error('Failed connecting to remote API server.');
    vrep.delete;
    return;
else
    disp("Connection open with id: "+ clientID);
end

