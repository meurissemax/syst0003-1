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
m = [1e+6, 3e+3];

% Stiffnesses and viscosities of the building and damper (N/m)
k = [power((2 * pi * f), 2) * m(1), 1e5];
c = [0.04 * m(1) * pi * f, 1e4];

% Building dimensions (m)
width = 250;
height = 30;

% Wind speed (m/s)
speed = 15;


%% Simulations

% Time (s)
t = 0:0.01:15;

% Initial conditions
x0 = [0.2, 1, 0.2, 1];