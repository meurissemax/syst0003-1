%% Linear control systems
%
% Study of an active mass damper
% Master in Civil Engineering
% University of Liège - Academic year 2019-2020
%
% Authors :
%   Bastien HOFFMANN
%   Maxime MEURISSE
%   Valentin VERMEYLEN

%% Initialization

% Close all opened figures
close all;

% Add path to Simulink files
addpath('simulink/');

% Get parameters
model.param;

% Get system
model.sys;

% Clear irrelevant variables
clearvars -except t x0 names types limits A B C D p r u d;
