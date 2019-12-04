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

ini;


%% Open loop system

% We change C and D matrices in order to have the four states as ouput
ol = ss(A, B, eye(size(A, 1)), zeros(size(A, 1), size(B, 2)));


%% Variables

% Get weak poles
wp = utils.weak(p);

% Xi and Omega
xi = [0.8, 0.8];
w_c = [5, 10 * 5];


%% State feedback controller design

% Get poles of K
p_ctrl = [
    wp(1), ...
    wp(2), ...
    (-xi(1) * w_c(1)) - (w_c(1) * sqrt(xi(1) * xi(1) - 1)), ...
    (-xi(1) * w_c(1)) + (w_c(1) * sqrt(xi(1) * xi(1) - 1)) ...
    ];

% Get K matrix
K = place(A, B(:, 2), p_ctrl);

% Get static gain
k_r = -1 / (C * ((A - B(:, 2) * K) \ B(:, 2)));


%% Observer design

% Get poles of L (add delays to original poles)
p_obs = [
    real(wp(1)) * 5 + imag(wp(1)) * 1i, ...
    real(wp(2)) * 5 + imag(wp(2)) * 1i, ...
    (-xi(2) * w_c(2)) - (w_c(2) * sqrt(xi(2) * xi(2) - 1)), ...
    (-xi(2) * w_c(2)) + (w_c(2) * sqrt(xi(2) * xi(2) - 1)), ...
    ];

% Get L matrix
L = place(A', C', p_obs)';


%% Simulations

% Simulate time response of the system (with Simulink)
out = sim('smlnk_hw3.slx', t);

y = out.yout{1}.Values.Data;
y_ctrl = out.yout{2}.Values.Data;
y_obs = out.yout{3}.Values.Data;


%% Plot simulations

% Controller
utils.graphic( ...
    t, ...
    {{d(:, 2)}, {y(:, 1), y_ctrl(:, 1)}}, ...
    'Time (s)', ...
    {'Amplitude (N)', types{1}}, ...
    {
        'Wind force', ...
        {
            strcat(names{1}, ' (natural oscillations)'), ...
            strcat(names{1}, ' (controlled oscillations)')
        }
    }, ...
    'hw3-controller' ...
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
    }, ...
    'hw3-observer' ...
);


%% Clear workspace

clearvars -except p ol K k_r L
