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

r = zeros(length(t), 1);


%% Controllable input

u = zeros(length(t),  1);


%% Uncontrollable input (wind)

% Maximal amplitude (N)
F_max = 0.6 * power(speed, 2) * width * height;

% Constant wind force
F_cst = F_max * ones(length(t), 1);

% Sinusoidal wind force
F_sin = F_max * sin(2 * pi * t);

% Random wind force
F_rand = F_max * rand(length(t), 1);

% Disturbance to study
d = F_cst;
