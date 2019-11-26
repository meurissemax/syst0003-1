%% Linear control systems
%
% Homework 3 - Controller in time domain
% Master in Civil Engineering
% University of Liège - Academic year 2019-2020
%
% Authors :
%   Bastien HOFFMANN
%   Maxime MEURISSE
%   Valentin VERMEYLEN

%% System modeling

% Modifiable parameters
f = 1; % Hz
m = [1e+8, 1e+3]; % kg
k = [(2 * pi * f) * (2 * pi * f) * m(1), 1e5]; % N/m
c = [2 * m(1) * 2 * pi * f * 0.01, 1e4]; % N/m

% State-space representation
A = [
    0, 1, 0, 0;
    (-k(1)-k(2))/m(1), (-c(2)-c(1))/m(1), k(2)/m(1), c(2)/m(1);
    0, 0, 0, 1;
    k(2)/m(2), c(2)/m(2), -k(2)/m(2), -c(2)/m(2)
    ];
B = [0, 0; 1/m(1), -1/m(1); 0, 0; 0, 1/m(2)];
C = eye(4);
D = [0, 0; 0, 0; 0, 0; 0, 0];

% System
sys = ss(A, B, C, D);


%% Controller variables

% Poles of A
p = sort(eig(A), 'descend');

% Xi and Omega
xi = zeros(1, 2);
w_c = zeros(1, 2);


%% State feedback controller design

% Modifiable parameters
xi(1) = 0.7;
w_c(1) = 10;

% Get poles of K
p_ctrl = [
    p(1), ...
    p(2), ...
    (-xi(1) * w_c(1)) - (w_c(1) * sqrt(xi(1) * xi(1) - 1)), ...
    (-xi(1) * w_c(1)) + (w_c(1) * sqrt(xi(1) * xi(1) - 1)) ...
    ];

% Get K matrix
K = place(A, B(:, 2), p_ctrl);

% Controlled system
sys_ctrl = ss(A - B(:, 2) * K, B, C, D);


%% Observer design

% Modifiable parameters
xi(2) = xi(1);
w_c(2) = 10 * w_c(1);

% Get poles of L
p_obs = [
    p(1), ...
    p(2), ...
    (-xi(2) * w_c(2)) - (w_c(2) * sqrt(xi(2) * xi(2) - 1)), ...
    (-xi(2) * w_c(2)) + (w_c(2) * sqrt(xi(2) * xi(2) - 1)), ...
    ];

% Get L matrix
L = place(A', C', p_obs)';

% Observer system
sys_obs = ss(A - L * C, B, C, D);


%% Simulations

% Wind force
F_max = 7350000; % N

% Time
t = 0:0.01:15; % s

% Controllable input
u(1:length(t), 1) = 0;

% Uncontrollable input
F_cst(1:length(t), 1) = F_max;
F_sin = F_max * sin(2 * pi * t');
F_rand = F_max * rand(length(t), 1);

F = F_cst;

% Open loop system
[y, ~, ~] = lsim(sys, [F, u], t);

% Controller
[y_ctrl, ~, ~] = lsim(sys_ctrl, [F, u], t);

% Observer
[y_obs, ~, ~] = lsim(sys_obs, [F, u], t);


%% Plot simulations

% Controller
figure('units', 'normalized', 'outerposition', [0 0 1 1]);
simplot( ...
    t, ...
    {{F}, {y(:, 1), y_ctrl(:, 1)}}, ...
    'Time (s)', ...
    {'Amplitude (N)', 'Displacement (m)'}, ...
    {{'Wind force'}, {'Natural oscillations', 'Controlled oscillations'}} ...
);

% Observer
figure('units', 'normalized', 'outerposition', [0 0 1 1]);
simplot( ...
    t, ...
    {
        {y_ctrl(:, 1), y_obs(:, 1)}, ...
        {y_ctrl(:, 2), y_obs(:, 2)}, ...
        {y_ctrl(:, 3), y_obs(:, 3)}, ...
        {y_ctrl(:, 4), y_obs(:, 4)} ...
    }, ...
    'Time (s)', ...
    {
        'Displacement (building) (m)', ...
        'Speed (building) (m/s)', ...
        'Displacement (damper) (m)', ...
        'Speed (damper) (m/s)'
    }, ...
    {
        {'Real state', 'Approximated state'}, ...
        {'Real state', 'Approximated state'}, ...
        {'Real state', 'Approximated state'}, ...
        {'Real state', 'Approximated state'} ...
    } ...
);


%% Clear workspace

clearvars -except f m k c sys xi w_c K sys_ctrl L sys_obs
