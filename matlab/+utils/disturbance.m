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

%% Function disturbance

function d = disturbance(d, t)

if length(d) ~= length(t)
    d = d(1, 1) * ones(length(t), 1);
end

end
