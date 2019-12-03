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

% Get parameters
utils.param;

% Get system
utils.sys;

% Close all opened figures
close all;

% Clear irrelevant variables
clearvars -except t x0 names types A B C D p r u d;
