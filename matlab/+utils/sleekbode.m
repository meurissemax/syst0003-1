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

%% Function sleekbode

function sleekbode(f)

% Initialize figure
figure;

% Bode plot
fig = bodeplot(f);

% Get options
options = getoptions(fig);

% Title
options.Title.String = '';

% Axis
options.XLabel.String = '$\omega$';
options.YLabel.String = {'Amplitude', 'Phase'};

options.XLabel.Interpreter = 'latex';
options.YLabel.Interpreter = 'latex';

% Grid
options.Grid = 'on';

% Font size
options.Title.FontSize = 12;
options.TickLabel.FontSize = 12;

% Save options
setoptions(fig, options);

end
