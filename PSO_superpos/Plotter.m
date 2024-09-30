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
            obj.initialize_plot(bounds, p_sources, n_drones, group_indices);
            obj.initialize_drones(n_drones, group_indices);
            obj.create_legend(group_indices);
        end
        
        % Initialize the main plot
        function initialize_plot(obj, bounds, p_sources, n_drones, group_indices)
            obj.fig = figure;
            obj.fig.Position = [220, 1, 1480, 1080];
            obj.bounds = bounds;
            obj.n_groups = max(group_indices);
            obj.colors = linspecer(obj.n_groups, 'qualitative');
            obj.scatter_sources = scatter(p_sources(:, 1), p_sources(:, 2), 300, 'red', '*');
            obj.ax = gca;
            obj.ax.XLim = [bounds(1), bounds(2)];
            obj.ax.YLim = [bounds(1), bounds(2)];
            xlabel('x'); ylabel('y');
            title('', 'FontSize', 15);
            grid on; grid minor;
            obj.plot_circle_lines(n_drones);
        end
        
        % Plot lines and circle for the boundaries and angular grid
        function plot_circle_lines(obj, n_drones)
            viscircles([0, 0], obj.bounds(2), 'LineStyle', '--', 'EdgeColor', 'black', 'LineWidth', 0.5);
            angles = compute_angles(n_drones);
            for i = 1:length(angles)
                angle = deg2rad(angles(i));
                line([0, obj.bounds(2) * cos(angle)], [0, obj.bounds(2) * sin(angle)], ...
                    'LineStyle', '--', 'Color', 'black', 'HandleVisibility', 'off');
            end
            hold on;
        end

        % Initialize drone positions and assign colors
        function initialize_drones(obj, n_drones, group_indices)
            obj.drone_colors = obj.assign_group_colors(n_drones, group_indices);
            initial_positions = zeros(n_drones, 2);
            obj.scatter_drones = scatter(initial_positions(:, 1), initial_positions(:, 2), 200, obj.drone_colors, 'filled');
        end
        
        % Assign colors to drones based on their group
        function drone_colors = assign_group_colors(obj, n_drones, group_indices)
            drone_colors = zeros(n_drones, 3);
            for i = 1:n_drones
                drone_colors(i, :) = obj.colors(group_indices(i), :);
            end
        end
        
        % Create the legend for the groups and sources
        function create_legend(obj, group_indices)
            obj.legend_entries = cell(1, obj.n_groups);
            obj.plot_handles = cell(1, obj.n_groups);
            obj.update_legend(group_indices);
        end
        
        % Update the drone plot with new positions
        function draw(obj, positions, iter)
            obj.scatter_drones.XData = positions(:, 1);
            obj.scatter_drones.YData = positions(:, 2);
            % Update the title
            title(obj.ax, sprintf('Iteration %d', iter));
            pause(0.03)
            drawnow;
        end

        % Plot an exclusion zone
        function plot_exclusion_zone(obj, radius, center)
            viscircles(center, radius, 'EdgeColor', 'k', 'LineWidth', 2, 'LineStyle', '--');
        end

        % Generate and append a new group color
        function generate_new_group_color(obj)
            new_colors = linspecer(obj.n_groups + 1, 'qualitative');
            new_color = new_colors(end, :);
            obj.colors = [obj.colors; new_color];
            obj.n_groups = obj.n_groups + 1;
        end

        % Update drone color based on its group
        function update_drone_color(obj, particle, group_indices)
            obj.generate_new_group_color();
            obj.drone_colors(particle.identifier, :) = obj.colors(end, :);
            obj.scatter_drones.CData(particle.identifier, :) = obj.drone_colors(particle.identifier, :);
            obj.update_legend(group_indices);
        end
        
        % Update the legend for new groups
        function update_legend(obj, group_indices)
            unique_groups = unique(group_indices);
            for i = 1:obj.n_groups
                obj.plot_handles{i} = scatter(NaN, NaN, 70, obj.colors(unique_groups(i), :), 'filled');
                obj.legend_entries{i} = sprintf('Drone(s) in Group %d', unique_groups(i));
            end
            legend([obj.scatter_sources, [obj.plot_handles{:}]], 'Sources', obj.legend_entries{:}, 'FontSize', 12, 'Location', 'northeastoutside');
        end

        % Final plot to show best positions with corresponding group colors
        function plot_best(obj, g_best_local, group_indices)
            hold on;
            for i = 1:size(g_best_local, 1)
                obj.plot_handles{i} = scatter(g_best_local(i, 1), g_best_local(i, 2), 350, obj.colors(group_indices(i), :), '*');
                obj.legend_entries{i} = sprintf('Best Estimate (Group %d)', group_indices(i));
            end
            legend([obj.scatter_sources, [obj.plot_handles{:}]], 'Sources', obj.legend_entries{:});
            title('Final Drone Positions and Group-Based Best Estimates', 'FontSize', 15);
            hold off;
        end
    end
end

