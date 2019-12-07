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


%% System modeling

% Natural frequency of the building (Hz)
f = 1;

% Masses of the building and damper (kg)
m = [1e+7, 3e+4];
m_tot = sum(m);

% Stiffnesses and viscosities of the building and damper (N/m)
k = power((2 * pi * f), 2) * m_tot;
c = 0.04 * m_tot * pi * f;


%% State-space representation

A = [
    0, 1;
    (-k)/m_tot, (-c)/m_tot;
    ];
B = [0, 0; 1/m_tot, -1/m_tot];
C = [1, 0];
D = [0, 0];

% Open loop system
sys = ss(A, B, C, D);


%% Simulation parameters

% Building dimensions (m)
width = 200;
height = 30;

% Wind speed (m/s)
speed = 15;

% Time
t = 0:0.01:15;

% Reference
r = [t', zeros(length(t), 1)];

% Controllable input (force on mass damper)
u = [t', zeros(length(t), 1)];

% Uncontrollable input (wind force)
F_max = 0.6 * power(speed, 2) * width * height; % N

F_zero = [t', zeros(length(t), 1)];
F_cst = [t', F_max * ones(length(t), 1)];
F_sin = [t', F_max * sin(2 * pi * t')];
F_rand = [t', F_max * rand(length(t), 1)];

d = F_sin;

% Initial conditions
x0 = [0, 0];


%% Transfer function

P = tf(sys);
P = P(2);

utils.sleekbode(P);

figure;
nyquist(P);


%% Lag compensator for amplification of low frequencies

a = 10;
g_lag = tf([1, a], [1, 0]);

utils.sleekbode(g_lag);


%% Lead compensator for the phase

phi = 80 * pi / 180;
omega_co = 20;
alpha = (pi / 4) - (phi / 2);

wp = omega_co / tan(alpha);
wz = omega_co * tan(alpha);

g_lead = tf([1 / wz, 1], [1 / wp, 1]);

utils.sleekbode(g_lead);


%% Low-pass filter

c = omega_co * 30;
lpf= tf(1, [1, c]);

utils.sleekbode(lpf);


%% Gain

gain = 1 / bode(g_lead * g_lag * P * lpf, omega_co);

utils.sleekbode(tf(gain, 1));


%% Controller design

C = g_lead * g_lag * lpf * gain;
L = C * P;

utils.sleekbode(-L);

figure;
nyquist(-L);


%% Gang of four

S = 1 / (1 - L); % Sensitivity function
PS = P / (1 - L); % Load sensitivity function
T = -L / (1 - L); % Complementary sensitivity function
CS = g_lead * gain * g_lag * lpf / (1 - L); % Noise sensitivity function

utils.sleekbode(S);
utils.sleekbode(PS);
utils.sleekbode(T);
utils.sleekbode(CS);


%% Simulations results (Simulink)

% Simulate and get results
out = sim('smlnk_hw4.slx', t);

y = out.yout{4}.Values.Data;
y_ctrl = out.yout{1}.Values.Data;
u_ctrl = out.yout{2}.Values.Data;
damper_a = out.yout{3}.Values.Data;

% Building displacement
utils.graphic( ...
    t, ...
    {{y(:, 1), y_ctrl(:, 1)}}, ...
    'Time (s)', ...
    {'Displacement (m)'}, ...
    {
        {
            'Building (natural oscillations)', ...
            'Building (controlled oscillations)'
        }
    }, ...
    'hw4-output' ...
);

% Controllable input
utils.graphic( ...
    t, ...
    {{u_ctrl}}, ...
    'Time (s)', ...
    {'Amplitude (N)'}, ...
    {'Controllable input'}, ...
    'hw4-controllable-input' ...
);

% Damper acceleration
utils.graphic( ...
    t, ...
    {{damper_a}}, ...
    'Time (s)', ...
    {'Acceleration ($m/s^2$)'}, ...
    {'Damper'}, ...
    'hw4-damper-acceleration' ...
);


%% Delay

t_d1 = 0.01;
t_d2 = 0.02;
t_d3 = 0.05;

S = tf('s');

sys1 = exp(-t_d1 * S);
sys2 = exp(-t_d2 * S);
sys3 = exp(-t_d3 * S);

L_del1 = ss(sys1);
L_del2 = ss(sys2); 
L_del3 = ss(sys3);

% Bode
figure;
hold on

bode(L * L_del1);
bode(L * L_del2);
bode(L * L_del3);

title('')
legend('t_d=0.03s', 't_d=0.05s', 't_d=0.07s')

hold off

% Nyquist
figure;
hold on

nyquist(-L * L_del1);
nyquist(-L * L_del2);
nyquist(-L * L_del3);

title('')
legend('t_d=0.03s', 't_d=0.05s', 't_d=0.07s')
axis([-1.5 1.5 -0.6 0.6])

hold off
