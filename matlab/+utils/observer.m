%% Linear control systems
%
% Study of an active mass damper
% Master in Civil Engineering
% University of Liège - Academic year 2019-2020
%
% Authors :
%   Bastien HOFFMANN
%   Maxime MEURISSE
%   Valentin VERMEYLE

%% Function observer

function L = observer(wp, reduce, alpha, zeta, w_c, A, C)

% Get poles of L (add delays to original poles)
p_obs = [
    real(wp(1)) * reduce * alpha + imag(wp(1)) * 1i, ...
    real(wp(2)) * reduce * alpha + imag(wp(2)) * 1i, ...
    (-zeta * w_c * alpha) - (w_c * alpha * sqrt(zeta * zeta - 1)), ...
    (-zeta * w_c * alpha) + (w_c * alpha * sqrt(zeta * zeta - 1)), ...
    ];

% Get L matrix
L = place(A', C', p_obs)';

end
