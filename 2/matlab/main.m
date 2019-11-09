%% Linear control systems
%
% Homework 2 - Open Loop System
% Master in Civil Engineering
% University of Liège - Academic year 2019-2020
%
% Authors :
%   Bastien HOFFMANN
%   Maxime MEURISSE
%   Valentin Vermeylen


%% System modeling

% Parameters
m = [2e+7 1e+4]; % mass (in kg)
k = [1.5e+9 3.5e+6]; % spring (in N/m)
c = [3e+7 1e+6]; % damper (in Ns/m)

% State space representation
A = [
    0 1 0 0;
    (-k(1)-k(2))/m(1) (-c(2)-c(1))/m(1) k(2)/m(1) c(2)/m(1);
    0 0 0 1;
    k(2)/m(2) c(2)/m(2) -k(2)/m(2) -c(2)/m(2)
    ];
B = [0; 1/m(1); 0; 0];
C = [1 0 0 0];
D = 0;

% System
sys = ss(A, B, C, D);


%% Question 4

% Parameters
building_height = 250; % building height (in m)
building_width = 40; % building width (in m)
area = building_height * building_width;

rho = 1.2; % air density (in kg/m^3)
wind_speed = 35; % wind speed (in m/s)

F_max = 0.5 * rho * wind_speed * wind_speed * area; % wind force (max value, in N)

t = 0:0.01:10; % time
f = 1; % frequency
w = 2 * pi * f; % omega

% Wind forces
F_cst(1, 1:length(t)) = F_max; % constant
F_sin = (F_max / 2) * sin(w * t) + (F_max / 2); % sinusoïdal (shifted)

% Simulation - Constant wind force
[y, t, ~] = lsim(sys, F_cst, t);

figure;
sleek_plot( ...
    t, ...
    {F_cst, y}, ...
    'Time (seconds)', ...
    {'Amplitude (N)', 'Displacement (m)'}, ...
    {'Wind force', 'building oscillation'}, ...
    true ...
);

% Simulation - Sinusoïdal wind force
[y, t, ~] = lsim(sys, F_sin, t);

figure;
sleek_plot( ...
    t, ...
    {F_sin, y}, ...
    'Time (seconds)', ...
    {'Amplitude (N)', 'Displacement (m)'}, ...
    {'Wind force', 'building oscillation'}, ...
    false ...
);


%% Q5 - a) Stability

% Poles
p = pole(sys);

if all(real(eig(A)) < 0)
    disp("System is stable");
else
    disp("System is not stable");
end


%% Q5 - b) Observability

% Observability matrix
Ob = obsv(sys);

% Number of unobservable states
unob = length(A) - rank(Ob);

disp(unob + " unobservable state(s)");


%% Q5 - c) Controllability

% Controllability matrix
Co = ctrb(sys);

% Number of uncontrollable states
unco = length(A) - rank(Co);

disp(unco + " uncontrollable state(s)");


%% Cleaning workspace

clearvars -except sys p Ob Co
