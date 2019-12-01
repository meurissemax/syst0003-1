%% Linear control systems
%
% Study of an active mass damper
% Master in Civil Engineering
% University of Li�ge - Academic year 2019-2020
%
% Authors :
%   Bastien HOFFMANN
%   Maxime MEURISSE
%   Valentin VERMEYLEN

%% System modeling

% Natural frequency of the building (Hz)
f = 1;

% Masses of the building and damper (kg)
m = [7e+5, 2e+3];

% Stiffnesses and viscosities of the building and damper (N/m)
k = [power((2 * pi * f), 2) * m(1), 1e4];
c = [0.04 * m(1) * pi * f, 1e3];

% Building dimensions (m)
width = 200;
height = 30;

% Wind speed (m/s)
speed = 10;


%% Simulations

% Time (s)
t = 0:0.01:15;

% Reference
r = zeros(length(t), 1);

% Initial conditions
x0 = [0.1, 0.5, 0.04, 0.1];
