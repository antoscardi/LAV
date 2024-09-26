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
        n_groups
    end
    
    methods
        % Constructor
        function obj = Plotter(p_sources, bounds, n_drones, group_indices)
            % Create a figure and set plot bounds
            obj.fig = figure;
            obj.fig.Position = [220, 1, 1480, 1080];
            obj.bounds = bounds;
            obj.n_groups = max(group_indices);
            obj.colors = linspecer(obj.n_groups, 'qualitative'); % Generate distinguishable colors for each group
            obj.drone_colors = zeros(n_drones, 3);
            obj.legend_entries = cell(1, obj.n_groups);
            obj.plot_handles = cell(1, obj.n_groups); % Initialize as a cell array for dynamic handling
            
            % Plot lines and circle
            viscircles([0, 0], bounds(2), 'LineStyle', '--', 'EdgeColor', 'black', 'LineWidth', 0.5);
            angles = compute_angles(n_drones);
            for i = 1:length(angles)
                angle = deg2rad(angles(i));
                line([0, bounds(2) * cos(angle)], [0, bounds(2) * sin(angle)], 'LineStyle', '--', 'Color', 'black', 'HandleVisibility', 'off');
            end
            hold on;
            
            % Map each drone's color based on its group index
            for i = 1:n_drones
                obj.drone_colors(i, :) = obj.colors(group_indices(i), :);  % Assign color based on group
            end
            
            % Initialize scatter plot for sources
            obj.scatter_sources = scatter(p_sources(:, 1), p_sources(:, 2), 300, 'red', '*'); % Mark sources in red
            
            % Initialize scatter plot for drones with their respective group colors
            initial_positions = zeros(n_drones, 2);
            obj.scatter_drones = scatter(initial_positions(:, 1), initial_positions(:, 2), 200, obj.drone_colors, 'filled');
            
            % Create the legend only once at initialization
            unique_groups = unique(group_indices);
            for i = 1:obj.n_groups
                obj.plot_handles{i} = scatter(NaN, NaN, 70, obj.colors(unique_groups(i), :), 'filled'); % Dummy points
                obj.legend_entries{i} = sprintf('Drone(s) in Group %d', unique_groups(i));
            end
            
            % Set plot limits and labels
            obj.ax = gca;
            obj.ax.XLim = [bounds(1), bounds(2)];
            obj.ax.YLim = [bounds(1), bounds(2)];
            xlabel('x'); ylabel('y');
            title('', 'FontSize', 15);
            grid on;
            grid minor;
            
            % Set the legend for sources and drone groups
            legend([obj.scatter_sources, [obj.plot_handles{:}]], 'Sources', obj.legend_entries{:}, 'FontSize', 12, 'Location', 'northeastoutside');
            drawnow;
        end

        % Update the plot with new drone positions, but do not update the legend
        function draw(obj, positions, iter)
            % Update drone positions on the plot
            obj.scatter_drones.XData = positions(:, 1);
            obj.scatter_drones.YData = positions(:, 2);
            
            % Update the title to reflect the current iteration
            titleString = sprintf('Iteration %d', iter);
            title(obj.ax, titleString);
            
            % Update the plot
            pause(0.02);
            drawnow;
        end

        function plot_exclusion_zone(obj, radius, center)
            % Plot the exclusion zone using viscircles
            viscircles(center, radius, 'EdgeColor', 'k', 'LineWidth', 2, 'LineStyle', '--');
        end
        
        % Generate and append new group color
        function obj = generate_new_group_color(obj)
            new_colors = linspecer(obj.n_groups + 1, 'qualitative');
            new_color = new_colors(end, :);  % Get the last color as the new color
            % Append the new color to the existing colors array
            obj.colors = [obj.colors; new_color];
        end

        % Function to update the drone color based on its group
        function update_drone_color(obj, particle, group_indices)
            obj.generate_new_group_color();
            obj.n_groups = obj.n_groups + 1;  % Update the total number of groups
            % Update the drone color in the scatter plot
            obj.drone_colors(particle.identifier, :) = obj.colors(end, :);
            obj.scatter_drones.CData(particle.identifier, :) = obj.drone_colors(particle.identifier, :);
            % Add a new group to the legend
            obj.add_new_group_legend(group_indices);
        end

        % Add a new group to the legend
        function add_new_group_legend(obj, group_indices)
            unique_groups = unique(group_indices);
            for i = 1:obj.n_groups
                obj.plot_handles{i} = scatter(NaN, NaN, 70, obj.colors(unique_groups(i), :), 'filled'); % Dummy points
                obj.legend_entries{i} = sprintf('Drone(s) in Group %d', unique_groups(i));
            end
            legend([obj.scatter_sources, [obj.plot_handles{:}]], 'Sources', obj.legend_entries{:}, 'FontSize', 12, 'Location', 'northeastoutside');
        end

        % Final plot to show best positions with corresponding group colors
        function plot_best(obj, g_best_local, group_indices)
            % Plot best estimates for each group using group-specific colors
            hold on;
            for i = 1:size(g_best_local, 1)
                obj.plot_handles{i} = scatter(g_best_local(i, 1), g_best_local(i, 2), 350, obj.colors(group_indices(i), :), '*');
                obj.legend_entries{i} = sprintf('Best Estimate (Group %d)', group_indices(i));
            end
            
            % Update the legend with sources and group best estimates
            legend([obj.scatter_sources, [obj.plot_handles{:}]], 'Sources', obj.legend_entries{:});
            title('Final Drone Positions and Group-Based Best Estimates', 'FontSize', 15);
            hold off;
        end
    end
end

