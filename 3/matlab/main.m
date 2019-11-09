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
C = [1, 0, 0, 0];
D = [0, 0];

% System
sys = ss(A, B, C, D);


%% State feedback controller design

% Modifiable parameters
xi_ctrl = 0.7;
w_c_ctrl = 10;

% Get poles of K
p = sort(eig(A), 'descend');
p_ctrl = [
    p(1), ...
    p(2), ...
    (-xi_ctrl * w_c_ctrl) - (w_c_ctrl * sqrt(xi_ctrl * xi_ctrl - 1)), ...
    (-xi_ctrl * w_c_ctrl) + (w_c_ctrl * sqrt(xi_ctrl * xi_ctrl - 1)) ...
    ];

% Get K matrix
K = place(A, B, p_ctrl);
K(1, :) = 0;


%% Observer design

% Modifiable parameters
xi_obs = xi_ctrl;
w_c_obs = 10 * w_c_ctrl;

% Get poles of L
p_obs = [
    p(1), ...
    p(2), ...
    (-xi_obs * w_c_obs) - (w_c_obs * sqrt(xi_obs * xi_obs - 1)), ...
    (-xi_obs * w_c_obs) + (w_c_obs * sqrt(xi_obs * xi_obs - 1)) ...
    ];

% Get L matrix
L = place(A', C', p_obs)';


%% Controlled system design

A_ctrl = [
    A - B * K, B * K;
    zeros(size(A)), A - L * C
    ];
B_ctrl = [B; zeros(size(B))];
C_ctrl = [C; zeros(size(C))];

sys_ctrl = ss(A_ctrl, B_ctrl, C_ctrl, D);


%% Simulations

% Modifiable parameters
t = transpose(0:0.1:15); % s

building = [250, 40]; % m
rho = 1.2; % kg/m^3
wind_speed = 35; % m/s

% Derived parameters
area = building(1, 1) * building(1, 2); % m^2
F_max = 0.5 * rho * wind_speed * wind_speed * area; % N

% Controllable input
u(1:length(t), 1) = 0;

% Uncontrollable input
F_cst(1:length(t), 1) = F_max;
F_sin = F_max * sin(2 * pi * t);
F_rand = F_max * rand(length(t), 1);

% Simulation (constant force)
[y, ~, ~] = lsim(sys, [F_cst, u], t);
[y_ctrl, ~, ~] = lsim(sys_ctrl, [F_cst, u], t);

figure;
simplot( ...
    t, ...
    {{F_cst}, {y, y_ctrl}}, ...
    'Time (s)', ...
    {'Amplitude (N)', 'Displacement (m)'}, ...
    {'Wind force', {'Natural oscillations', 'Controlled oscillations'}} ...
);

% Simulation (sinusoidal force)
[y, ~, ~] = lsim(sys, [F_sin, u], t);
[y_ctrl, ~, ~] = lsim(sys_ctrl, [F_sin, u], t);

figure;
simplot( ...
    t, ...
    {{F_sin}, {y, y_ctrl}}, ...
    'Time (s)', ...
    {'Amplitude (N)', 'Displacement (m)'}, ...
    {'Wind force', {'Natural oscillations', 'Controlled oscillations'}} ...
);

% Simulation (random force)
[y, ~, ~] = lsim(sys, [F_rand, u], t);
[y_ctrl, ~, ~] = lsim(sys_ctrl, [F_rand, u], t);

figure;
simplot( ...
    t, ...
    {{F_rand}, {y, y_ctrl}}, ...
    'Time (s)', ...
    {'Amplitude (N)', 'Displacement (m)'}, ...
    {'Wind force', {'Natural oscillations', 'Controlled oscillations'}} ...
);


%% Clear workspace

%clearvars -except f m k c sys xi w_c K sys_ctrl F_max
