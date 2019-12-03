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

%% States

% Names of the elements for each state
names = {'Building', 'Building', 'Damper', 'Damper'};

% Types of the states
types = {'Displacement (m)', 'Speed (m/s)', 'Displacement (m)', 'Speed (m/s)'};

% Limits for damper states
limits = [5, -5; 5, -5];


%% State-space representation

A = [
    0, 1, 0, 0;
    (-k(1)-k(2))/m(1), (-c(2)-c(1))/m(1), k(2)/m(1), c(2)/m(1);
    0, 0, 0, 1;
    k(2)/m(2), c(2)/m(2), -k(2)/m(2), -c(2)/m(2)
    ];
B = [0, 0; 1/m(1), -1/m(1); 0, 0; 0, 1/m(2)];
C = eye(4);
D = zeros(4, 2);


%% Dynamic of the system

% Poles
p = eig(A);


%% Reference

r = 0;


%% Inputs

% Controllable input
u = 0;

% Uncontrollable input (wind force)
F_max = 0.6 * power(speed, 2) * width * height; % N
