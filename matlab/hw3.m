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

%% Initialization

ini;


%% Open loop system

ol = ss(A, B, C, D);


%% Variables

% Get weak poles
wp = utils.weak(p);

% Xi and Omega
xi = zeros(1, 2);
w_c = zeros(1, 2);

% k_r
k_r = 1;


%% State feedback controller design

% Pole placement
xi(1) = 0.8;
w_c(1) = 10;

% Get poles of K
p_ctrl = [
    wp(1), ...
    wp(2), ...
    (-xi(1) * w_c(1)) - (w_c(1) * sqrt(xi(1) * xi(1) - 1)), ...
    (-xi(1) * w_c(1)) + (w_c(1) * sqrt(xi(1) * xi(1) - 1)) ...
    ];

% Get K matrix
K = place(A, B(:, 2), p_ctrl);
K = [zeros(1, length(K)); K];


%% Observer design

% Pole placement
xi(2) = xi(1);
w_c(2) = 10 * w_c(1);

% Get poles of L (add delays to original poles)
p_obs = [
    real(wp(1)) * 10 + imag(wp(1)) * 1i, ...
    real(wp(2)) * 10 + imag(wp(2)) * 1i, ...
    (-xi(2) * w_c(2)) - (w_c(2) * sqrt(xi(2) * xi(2) - 1)), ...
    (-xi(2) * w_c(2)) + (w_c(2) * sqrt(xi(2) * xi(2) - 1)), ...
    ];

% Get L matrix
L = place(A', C', p_obs)';

% Observer system
obs = ss(A - L * C, [B, L], C, zeros(4, 6));


%% Simulations

% Simulate time response of the system (with Simulink)
out = sim('smlnk_hw3.slx', t);

d = out.yout{1}.Values.Data;
d = utils.disturbance(d, t);

y = out.yout{2}.Values.Data;
y_ctrl = out.yout{3}.Values.Data;
y_obs = out.yout{4}.Values.Data;


%% Plot simulations

% Controller
utils.graphic( ...
    t, ...
    {{d}, {y(:, 1), y_ctrl(:, 1)}}, ...
    'Time (s)', ...
    {'Amplitude (N)', types{1}}, ...
    {
        'Wind force', ...
        {
            strcat(names{1}, ' (natural oscillations)'), ...
            strcat(names{1}, ' (controlled oscillations)')
        }
    } ...
);

% Observer
utils.graphic( ...
    t, ...
    {
        {y_ctrl(:, 1), y_obs(:, 1)}, ...
        {y_ctrl(:, 2), y_obs(:, 2)}, ...
        {y_ctrl(:, 3), y_obs(:, 3)}, ...
        {y_ctrl(:, 4), y_obs(:, 4)}
    }, ...
    'Time (s)', ...
    {types{1}, types{2}, types{3}, types{4}}, ...
    {
        {
            strcat(names{1}, ' (real)'), ...
            strcat(names{1}, ' (observer)')
        }, ...
        {
            strcat(names{2}, ' (real)'), ...
            strcat(names{2}, ' (observer)')
        }, ...
        {
            strcat(names{3}, ' (real)'), ...
            strcat(names{3}, ' (observer)')
        }, ...
        {
            strcat(names{4}, ' (real)'), ...
            strcat(names{4}, ' (observer)')
        }
    } ...
);


%% Clear workspace

clearvars -except x0 p r u d ol wp xi w_c k_r K L obs;
