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
    
    colors = ['b', 'g'];
    
    
    %% Subplots
    
    for i = 1:length(y)
        if length(y) > 2 && mod(length(y), 2) == 0
            subplot(length(y) / 2, length(y) / 2, i);
        else
            subplot(length(y), 1, i);
        end
        
        % Plot data
        hold on
        
        for j = 1:length(y{i})
            plot(x, y{i}{j}, 'LineWidth', plot_linewidth, 'Color', colors(mod(j, length(y{i}) + 1)));
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
