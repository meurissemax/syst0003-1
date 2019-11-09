%% Linear control systems
%
% Homework 3 - Controller in time domain
% Master in Civil Engineering
% University of Liège - Academic year 2019-2020
%
% Authors :
%   Bastien HOFFMANN
%   Maxime MEURISSE
%   Valentin VERMEYLEN

%% Function simplot

function simplot(x, y, x_label, y_label, plot_legend)
    %% Parameters
    
    plot_linewidth = 2;
    label_fontsize = 16;
    ax_fontsize = 12;
    legend_fontsize = 14;
    
    colors = {{'r'}, {'b', 'g'}};
    
    
    %% Subplots
    
    for i = 1:length(y)
        subplot(length(y), 1, i);
        
        hold on
        
        % Plot data
        for j = 1:length(y{i})
            plot( ...
            x, ...
            y{i}{j}, ...
            'LineWidth', plot_linewidth, ...
            'Color', colors{mod(i, length(colors) + 1)}{mod(j, length(colors{i}) + 1)} ...
            );
        end
        
        hold off
        
        % Axes
        xlabel(x_label, 'FontSize', label_fontsize);
        ylabel(y_label{i}, 'FontSize', label_fontsize);

        % Axes options
        ax = gca;
        ax.FontSize = ax_fontsize;

        % Legend
        legend(plot_legend{i}, 'Location', 'northeast', 'FontSize', legend_fontsize);

        % Others
        grid on
    end
end
