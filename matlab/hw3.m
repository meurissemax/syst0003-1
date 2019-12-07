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

% Zeta and Omega values to test
zeta = [0.2, 0.8, 1.3];
w_c = [10, 100, 1000];

% Multiplicative constant to reduce system rapidity
reduce = 0.5;

% Multiplicative constant for the observer
alpha = 5;


%% Choice of parameters Zeta and Omega

u_ctrl = cell(1, length(zeta));
u_legend = cell(1, length(zeta));

% Simulate for multiple coupe of (zeta, w_c) values
for i = 1:length(zeta)
    [K, k_r] = utils.controller(wp, reduce, zeta(i), w_c(i), A, B, C); %#ok<ASGLU>
    L = utils.observer(wp, reduce, alpha, zeta(i), w_c(i), A, C); %#ok<NASGU>
    
    out = sim('smlnk_hw3.slx', t);
    
    u_ctrl{i} = out.yout{1}.Values.Data;
    u_legend{i} = strcat('$\zeta = ', num2str(zeta(i)), '$, $\omega_c = ', num2str(w_c(i)), '$');
end

% Plot simulation results
utils.graphic( ...
    t, ...
    {u_ctrl}, ...
    'Time (s)', ...
    {'Amplitude (N)'}, ...
    {u_legend}, ...
    'hw3-zeta-omega' ...
);

% Final values chosen
zeta = 0.8;
w_c = 10;


%% State feedback controller design

[K, k_r] = utils.controller(wp, reduce, zeta, w_c, A, B, C);


%% Observer design

L = utils.observer(wp, reduce, alpha, zeta, w_c, A, C);


%% Simulations

% Simulate time response of the system (with Simulink)
out = sim('smlnk_hw3.slx', t);

u_ctrl = out.yout{1}.Values.Data;

y = out.yout{2}.Values.Data;
y_ctrl = out.yout{3}.Values.Data;
y_obs = out.yout{4}.Values.Data;


%% Plot simulations

% Controllable input (force on mass damper)
utils.graphic( ...
    t, ...
    {{u_ctrl}}, ...
    'Time (s)', ...
    {'Amplitude (N)'}, ...
    {'Controllable input'}, ...
    'hw3-controllable-input' ...
);

% Controller
utils.graphic( ...
    t, ...
    {{u_ctrl}, {y(:, 1), y_ctrl(:, 1)}}, ...
    'Time (s)', ...
    {'Amplitude (N)', types{1}}, ...
    {
        'Control input', ...
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
