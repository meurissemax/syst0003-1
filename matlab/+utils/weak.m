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

%% Function weak

function wp = weak(p)

% Tolerence between weakt pole and others possible weak poles
tol = 1e-3;

% Find weak poles
[~, ind] = max(abs(real(p)));
wp = p(abs(real(p - p(ind))) <= tol);

end
