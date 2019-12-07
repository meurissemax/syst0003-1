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

%% Function controller

function [K, k_r] = controller(wp, reduce, zeta, w_c, A, B, C)

% Get poles of K
p_ctrl = [
    real(wp(1)) * reduce + imag(wp(1)) * 1i, ...
    real(wp(2)) * reduce + imag(wp(2)) * 1i, ...
    (-zeta * w_c) - (w_c * sqrt(zeta * zeta - 1)), ...
    (-zeta * w_c) + (w_c * sqrt(zeta * zeta - 1)) ...
    ];

% Get K matrix
K = place(A, B(:, 2), p_ctrl);

% Get static gain
k_r = -1 / (C * ((A - B(:, 2) * K) \ B(:, 2)));

end
