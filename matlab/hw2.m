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

ol = ss(A, B, C, D);


%% Simulations

% Simulate time response of the system (with Simulink)
out = sim('smlnk_hw2.slx', t);

d = out.yout{1}.Values.Data;
d = utils.disturbance(d, t);

y = out.yout{2}.Values.Data;

% Plot first state
utils.graphic( ...
    t, ...
    {{d}, {y(:, 1)}}, ...
    'Time (s)', ...
    {'Amplitude (N)', types{1}}, ...
    {'Wind force', names{1}} ...
);

% Plot all states
utils.graphic( ...
    t, ...
    {{y(:, 1)}, {y(:, 2)}, {y(:, 3)}, {y(:, 4)}}, ...
    'Time (s)', ...
    {types{1}, types{2}, types{3}, types{4}}, ...
    {names{1}, names{2}, names{3}, names{4}} ...
);


%% Stability

if all(real(p) < 0)
    disp("System is stable");
else
    disp("System is not stable");
end


%% Observability

Ob = obsv(ol);
unob = length(ol.A) - rank(Ob);

disp(unob + " unobservable state(s)");


%% Controllability

Co = ctrb(ol.A, ol.B(:, 2));
unco = length(ol.A) - rank(Co);

disp(unco + " uncontrollable state(s)");


%% Clear irrelevant variables

clearvars -except p ol Ob Co;
