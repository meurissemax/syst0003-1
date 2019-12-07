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
m = [1e+7, 3e+3]; % kg
m_tot = m(1) + m(2);
k = power((2 * pi * f), 2) * m_tot; % N/m
c = 0.04 * m_tot * pi * f; % N/m

% State-space representation
A = [
    0, 1;
    (-k)/m_tot, (-c)/m_tot;
    ];
B = [0, 0; 1/m_tot, -1/m_tot];
C = [1, 0];
D = [0 0];
% System
sys = ss(A, B, C, D);
%% Simulations

% Wind force
F_max = 1012500; % N

% Time
t = 0:0.01:15; % s

% Reference
u = zeros(length(t), 1);

% Uncontrollable input
F_cst(1:length(t), 1) = F_max;
F_sin = F_max * sin(2 * pi * t');
F_rand = F_max * rand(length(t), 1);

F = F_cst;

% Open loop system
%[y, ~, ~] = lsim(sys, [F, u], t);
%plot(t, y);

P = tf(sys);
P = P(2);

%% Integrator and derivator

I = 500 * tf([1],[1 0]);
Z = 10000 * tf([1 0],[1]);


Gain = tf(1000000000, 1);
%% Lead compensator for the phase and Gain

phi = 1;
omega_co = 20;%2*pi*2;
alpha = pi/4 - phi/2;
Gl = 1;

wp = omega_co/tan(alpha);
wz = omega_co*tan(alpha);

Glead = Gl * tf([1/wz 1],[1/wp 1]);

G_Lo = 1/bode(Glead*P,omega_co);

%% Controller

L =Glead*P*G_Lo;
bode(-L);

%% Gang of 4

s = 1/(1-L); %sensitivity
ps = P/(1-L); %load
T = -L/(1-L); %complementary
cs = I*Z*Glead/(1-L); %complementary

% 
t_d1=0.01
t_d2=0.02;
t_d3=0.05;
s = tf('s');
sys1 = exp(-t_d1*s)
sys2 = exp(-t_d2*s)
sys3 = exp(-t_d3*s)
L_del1=ss(sys1);
L_del2= ss(sys2); 
L_del3=ss(sys3);

[Gm_del1, Pm_del1, Wcg_del1, Wcp_del1]=margin(L_del1);
[Gm_del2, Pm_del3, Wcg_del2, Wcp_del2]=margin(L_del2);
[Gm_del2, Pm_del3, Wcg_del3, Wcp_del3]=margin(L_del3);

figure
hold on
bodeplot(L*L_del1)
bodeplot(L*L_del2)
bodeplot(L*L_del3)
title('')
  legend('t_d=0.03s', 't_d=0.05s', 't_d=0.07s')
hold off

figure
hold on
nyquist(-L*L_del1);
nyquist(-L*L_del2)
nyquist(-L*L_del3)
title('')
legend('t_d=0.03s', 't_d=0.05s', 't_d=0.07s')
axis([-1.5 1.5 -0.6 0.6])
hold off
