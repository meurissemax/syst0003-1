%% Linear control systems
%
% Homework 2 - Open Loop System
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

% Simulation (initial conditions)
[y, ~, ~] = initial(sys, [0.5, 0, 0, 0], t);

figure;
simplot( ...
    t, ...
    {y}, ...
    'Time (s)', ...
    {'Displacement (m)'}, ...
    {'Building oscillation'} ...
);

% Simulation (constant force)
[y, ~, ~] = lsim(sys, [F_cst, u], t);

figure;
simplot( ...
    t, ...
    {F_cst, y}, ...
    'Time (s)', ...
    {'Amplitude (N)', 'Displacement (m)'}, ...
    {'Wind force', 'Building oscillation'} ...
);

% Simulation (sinusoidal force)
[y, ~, ~] = lsim(sys, [F_sin, u], t);

figure;
simplot( ...
    t, ...
    {F_sin, y}, ...
    'Time (s)', ...
    {'Amplitude (N)', 'Displacement (m)'}, ...
    {'Wind force', 'Building oscillation'} ...
);

% Simulation (random force)
[y, t, ~] = lsim(sys, [F_rand, u], t);

figure;
simplot( ...
    t, ...
    {F_rand, y}, ...
    'Time (s)', ...
    {'Amplitude (N)', 'Displacement (m)'}, ...
    {'Wind force', 'Building oscillation'} ...
);


%% Stability

p = pole(sys);

if all(real(p) < 0)
    disp("System is stable");
else
    disp("System is not stable");
end


%% Observability

Ob = obsv(A, C);
unob = length(sys.A) - rank(Ob);

disp(unob + " unobservable state(s)");


%% Controllability

Co = ctrb(A, B(:, 2));
unco = length(sys.A) - rank(Co);

disp(unco + " uncontrollable state(s)");


%% Clear workspace

clearvars -except f m k c sys F_max p Ob Co
