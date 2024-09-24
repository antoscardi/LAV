classdef Plotter < handle
    properties
        fig
        scatter_drones
        scatter_sources
        ax
        bounds
        colors
        drone_colors
        legend_entries
        plot_handles
    end
    
    methods
        % Constructor
        function obj = Plotter(p_sources, bounds, n_drones, group_indices, n_groups, initial_positions)
            % Create a figure and set plot bounds
            obj.fig = figure;
            obj.fig.Position = [220, 1, 1480, 1080];
            obj.bounds = bounds;
            obj.colors = linspecer(n_groups, 'qualitative'); % Generate distinguishable colors for each group
            obj.drone_colors = zeros(n_drones, 3);
            obj.legend_entries = cell(1, n_groups);
            obj.plot_handles = zeros(1, n_groups);
            
            % Map each drone's color based on its group index
            for i = 1:n_drones
                obj.drone_colors(i, :) = obj.colors(group_indices(i), :);  % Assign color based on group
            end
            
            % Initialize scatter plot for sources
            obj.scatter_sources = scatter(p_sources(:,1), p_sources(:,2), 300, 'red', '*'); % Mark sources in red
            hold on;
            
            % Initialize scatter plot for drones with their respective group colors
            obj.scatter_drones = scatter(initial_positions(:,1), initial_positions(:,2), 200, obj.drone_colors, 'filled');
            
            % Create the legend only once at initialization
            unique_groups = unique(group_indices);
            n_groups = length(unique_groups);
            for i = 1:n_groups
                obj.plot_handles(i) = scatter(NaN, NaN, 70, obj.colors(unique_groups(i), :), 'filled'); % Dummy points
                obj.legend_entries{i} = sprintf('Drone(s) in Group %d', unique_groups(i));
            end
            
            % Set plot limits and labels
            obj.ax = gca;
            obj.ax.XLim = [bounds(1) bounds(2)];
            obj.ax.YLim = [bounds(1) bounds(2)];
            xlabel('x'); ylabel('y');
            title('','FontSize', 15);
            grid on;
            grid minor;
            legend('FontSize', 12, 'Location', 'northeastoutside', 'Orientation', 'vertical');
            
            % Set the legend for sources and drone groups
            legend([obj.scatter_sources, obj.plot_handles], 'Sources', obj.legend_entries{:});
            
            drawnow;
        end

        % Update the plot with new drone positions, but do not update the legend
        function draw(obj, positions, iter, ~)
            % Update drone positions on the plot
            obj.scatter_drones.XData = positions(:,1);
            obj.scatter_drones.YData = positions(:,2);
            
            % Update the title to reflect the current iteration
            titleString = sprintf('Iteration %d', iter);
            title(obj.ax, titleString);
            
            % Update the plot
            pause(0.02);
            drawnow;
        end
        
        % Final plot to show best positions with corresponding group colors
        function plot_best(obj, g_best_local, group_indices)
            % Plot best estimates for each group using group-specific colors
            hold on;
            
            for i = 1:size(g_best_local, 1)
                % Plot best estimate with the group's corresponding color
                obj.plot_handles(i) = scatter(g_best_local(i, 1), g_best_local(i, 2), 350, obj.colors(group_indices(i), :), '*');
                obj.legend_entries{i} = sprintf('Best Estimate (Group %d)', group_indices(i));
            end
            
            % Update the legend with sources and group best estimates
            legend([obj.scatter_sources, obj.plot_handles], 'Sources', obj.legend_entries{:});
            
            title('Final Drone Positions and Group-Based Best Estimates','FontSize', 15);
            hold off;
        end
    end
end


