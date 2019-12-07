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

%% Function graphic

function graphic(x, y, x_label, y_label, plot_legend, fig_name)
    % Parameters
    set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
    set(groot, 'defaultLegendInterpreter', 'latex');
    
    pt = 12; % font size
    ptx = 20; % width
    pty = 12; % height
    lw = 1.5; % line width
    colors = ['b', 'g', 'r', 'c', 'm'];
    
    % Initialize figure
    figure;
    
    % Subplots
    for i = 1:length(y)
        if length(y) > 2 && mod(length(y), 2) == 0
            subplot(length(y) / 2, length(y) / 2, i);
        else
            subplot(length(y), 1, i);
        end
        
        % Plot data
        hold on
        
        for j = 1:length(y{i})
            plot(x, y{i}{j}, 'LineWidth', lw, 'Color', colors(mod(j, length(y{i}) + 1)));
        end
        
        hold off
        
        % Axes
        xlabel(x_label, 'interpreter', 'latex');
        ylabel(y_label{i}, 'interpreter', 'latex');
        
        % Legend
        if ~isempty(plot_legend)
            legend(plot_legend{i}, 'Location', 'northeast');
        end

        % Axes options
        ax = gca;
        ax.FontSize = pt;
        ax.YAxis.Exponent = 0;

        % Others
        grid on
        box off
    end
    
    % Export figure
    set(gcf, 'PaperPositionMode', 'auto');
    set(gcf, 'PaperUnits', 'centimeters');
    set(gcf, 'PaperPosition', [0, 0, ptx, pty]);
    
    print(gcf, strcat('figures/', fig_name), '-depsc')
end
