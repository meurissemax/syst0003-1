%% Linear control systems
%
% Homework 2 - Open Loop System
% Master in Civil Engineering
% University of Liège - Academic year 2019-2020
%
% Authors :
%   Bastien HOFFMANN
%   Maxime MEURISSE
%   Valentin Vermeylen


%% Function sleek_plot
%
% This function is used to plot multiple figures in a sleek way.

function sleek_plot(x, y, x_label, y_label, plot_legend, y_lim)
    %% Parameters
    plot_linewidth = 2;
    label_fontsize = 16;
    ax_fontsize = 12;
    legend_fontsize = 14;
    
    
    %% First subplot
    subplot(2, 1, 1);
    
    % Plot data
    plot(x, y{1}, 'LineWidth', plot_linewidth, 'Color', 'b');
    
    % Axes
    xlabel(x_label, 'FontSize', label_fontsize);
    ylabel(y_label{1}, 'FontSize', label_fontsize);
    
    if y_lim
        y_max = max(y{1});
        ylim([y_max - (y_max / 2), y_max + (y_max / 2)]);
    end
    
    % Axes options
    ax = gca;
    ax.FontSize = ax_fontsize;
    
    % Legend
    legend(plot_legend{1}, 'Location', 'northeast', 'FontSize', legend_fontsize);
    
    % Others
    grid on
    
    
    %% Second subplot
    subplot(2, 1, 2);
    
    % Plot data
    plot(x, y{2}, 'LineWidth', plot_linewidth, 'Color', 'r');
    
    % Axes
    xlabel(x_label, 'FontSize', label_fontsize);
    ylabel(y_label{2}, 'FontSize', label_fontsize);
    
    % Axes options
    ax = gca;
    ax.FontSize = ax_fontsize;
    ax.YRuler.Exponent = 0;
    
    % Legend
    legend(plot_legend{2}, 'Location', 'northeast', 'FontSize', legend_fontsize);
    
    % Others
    grid on
end
