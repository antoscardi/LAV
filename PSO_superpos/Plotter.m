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
        font_labels = 30;
        font_title = 30;
        font_legend = 26;
        do_video = true;
        video_filename = 'figures/case_1.avi';
        video
    end
    
    methods
        % Constructor
        function obj = Plotter(p_sources, bounds, n_drones, group_indices)
            obj.initialize_plot(bounds, p_sources, n_drones, group_indices);
            obj.initialize_drones(n_drones, group_indices);
            obj.create_legend(group_indices);
            if obj.do_video
                obj.video = VideoWriter('figures/case_1', 'Uncompressed AVI');
                obj.video.FrameRate = 5 ; % Match simulation time step
                open(obj.video);
            end
        end
        
        % Initialize the main plot
        function initialize_plot(obj, bounds, p_sources, n_drones, group_indices)
            obj.fig = figure;
            obj.fig.Position = [220, 1, 1480, 1080];
            obj.bounds = bounds;
            obj.n_groups = max(group_indices);
            obj.colors = linspecer(obj.n_groups, 'qualitative');
            obj.scatter_sources = scatter(p_sources(:, 1), p_sources(:, 2), 300, 'red', '*', 'DisplayName', 'Sources');
            obj.ax = gca;
            obj.ax.XLim = [bounds(1), bounds(2)];
            obj.ax.YLim = [bounds(1), bounds(2)];
            xlabel('x [m]', 'Interpreter', 'latex', 'FontSize', obj.font_labels);
            ylabel('y [m]', 'Interpreter', 'latex', 'FontSize', obj.font_labels);
            title('', 'FontSize', obj.font_title, 'Interpreter', 'latex');
            axis equal;
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
        function draw(obj, positions, iter, time)
            obj.scatter_drones.XData = positions(:, 1);
            obj.scatter_drones.YData = positions(:, 2);
            % Update the title
            title(obj.ax, sprintf('Iteration: %d, Simulation Time: %.1f s', iter, time), ...
                'Interpreter', 'latex', 'FontSize', obj.font_title);
            if obj.do_video
                frame = getframe(obj.fig); % Capture high-resolution frame
                %frame = imresize(frame.cdata, [995, 1910]);
                writeVideo(obj.video, frame);
            end
            pause(0.04);
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
            legend([obj.scatter_sources, [obj.plot_handles{:}]], 'Sources', obj.legend_entries{:}, ...
                'FontSize', obj.font_legend, 'Location', 'northeastoutside', 'Interpreter', 'latex');
        end

        % Final plot to show best positions with corresponding group colors
        function plot_best(obj, g_best_local, group_indices)
            hold on;
            for i = 1:size(g_best_local, 1)
                obj.plot_handles{i} = scatter(g_best_local(i, 1), g_best_local(i, 2), 350, obj.colors(group_indices(i), :), '*');
                obj.legend_entries{i} = sprintf('Best Estimate (Group %d)', group_indices(i));
            end
            legend([obj.scatter_sources, [obj.plot_handles{:}]], 'Sources', obj.legend_entries{:}, ...
                'FontSize', obj.font_legend, 'Interpreter', 'latex');
            title('Final Drone Positions and Group-Based Best Estimates', 'FontSize', obj.font_title, 'Interpreter', 'latex');
            hold off;
            if obj.do_video
                close(obj.video);
                disp(['High-quality video saved as ', obj.video_filename]);
            end
        end

        % Method to plot trajectories
        function plot_trajectories(obj, trajectories, valid_steps, n_drones)
            hold on;
            % Plot trajectories for all drones
            for i = 1:n_drones
                % Extract valid trajectory for the current drone
                drone_trajectory = squeeze(trajectories(i, 1:valid_steps(i), :)); % Only valid rows
                
                % Plot the trajectory
                plot(drone_trajectory(:, 1), drone_trajectory(:, 2), '-', 'LineWidth', 3, ...
                        'DisplayName', sprintf('Trajectory Drone %d', i), 'Color', obj.drone_colors(i, :));
            end
            
            % Add labels, title, and legend
            xlabel('x [m]', 'Interpreter', 'latex', 'FontSize', obj.font_labels);
            ylabel('y [m]', 'Interpreter', 'latex', 'FontSize', obj.font_labels);
            title('Drone Trajectories During PSO', 'FontSize', obj.font_title, 'Interpreter', 'latex');
            legend('FontSize', obj.font_legend, 'Location', 'northeastoutside', 'Interpreter', 'latex');
            grid on;
            a = annotation('rectangle',[0 0 1 1],'Color','w');
            exportgraphics(gcf, fullfile('figures', 'case_3.pdf'),'ContentType', 'vector');
            delete(a);
            hold off;
        end

        function plot_rotor_vel(obj, rotor_velocities_history, valid_steps, n_drones, dt)
            % Define colors
            red = [139, 0, 0] / 255;      % Red color (RGB: 139,0,0)
            orange = [0.922, 0.38, 0.24]; % Orange color (hex: #eb613d)
            blue = [0, 0, 205] / 255;     % Blue color (RGB: 0,0,205)
            green = [0, 128, 0] / 255;    % Green color (RGB: 0,128,0)
        
            % Create figure with Full HD dimensions
            figure('Position', [100, 100, 1920, 1080]); % Full HD dimensions
        
            % Initialize handles for legend
            legend_handles = [];
            legend_labels = {'$\omega_1(t)$', '$\omega_2(t)$', '$\omega_3(t)$', '$\omega_4(t)$'};
        
            % Set up tiled layout with adjusted spacing
            tiledlayout(n_drones, 1, 'TileSpacing', 'Compact', 'Padding', 'Compact');
        
            % Loop through each drone to create subplots
            for i = 1:n_drones
                % Extract rotor velocities for this drone for all valid steps
                drone_rotor_velocities = squeeze(rotor_velocities_history(i, 1:valid_steps(i), :)); % Dimensions: [valid_steps, 4]
                
                % Generate time steps for valid steps
                time_steps = (1:valid_steps(i)) * dt; % Multiply by time step to get time in seconds
        
                % Create an individual subplot
                nexttile;
                hold on;
                h1 = plot(time_steps, drone_rotor_velocities(:, 1), 'Color', red, 'LineWidth', 1.5); % Rotor 1
                h2 = plot(time_steps, drone_rotor_velocities(:, 2), 'Color', orange, 'LineWidth', 1.5); % Rotor 2
                h3 = plot(time_steps, drone_rotor_velocities(:, 3), 'Color', blue, 'LineWidth', 1.5); % Rotor 3
                h4 = plot(time_steps, drone_rotor_velocities(:, 4), 'Color', green, 'LineWidth', 1.5); % Rotor 4
                hold off;
        
                % Collect handles only for the first subplot
                if i == 1
                    legend_handles = [h1, h2, h3, h4];
                end
        
                % Set axis labels and title with LaTeX formatting
                xlabel_handle = xlabel('$t$ [s]', 'Interpreter', 'latex', 'FontSize', obj.font_labels-8);
                ylabel('$\omega_i$ [rad/s]', 'Interpreter', 'latex', 'FontSize', obj.font_labels-8);
                title(sprintf('Drone %d', i), 'Interpreter', 'latex', 'FontSize', obj.font_title-5);
        
                % Dynamically move x-axis label to the end of the plot
                x_limits = xlim; % Get x-axis limits
                xlabel_handle.Position(1) = x_limits(2); % Set the label position to the end of the x-axis
                xlabel_handle.HorizontalAlignment = 'right'; % Align the label to the right for better appearance

                % Add grid
                grid on;
            end
        
            % Add a global title
            sgtitle('Control inputs $\omega_i(t)$', 'Interpreter', 'latex', 'FontSize', obj.font_title-3);
        
            % Add a single legend outside the plotting area
            lg = legend(legend_handles, legend_labels, 'Interpreter', 'latex', 'FontSize', obj.font_legend, 'Orientation', 'horizontal');
            lg.Layout.Tile = 'north'; % Place the legend above the subplots
        
            % Add the annotation box
            a = annotation('rectangle', [0 0 1 1], 'Color', 'w');
        
            % Export the figure
            exportgraphics(gcf, fullfile('figures', 'rotor_vels_pso.pdf'), 'ContentType', 'vector');
        
            % Remove the annotation box
            delete(a);
        end

        function plot_comparison(obj, input_data, real_data, n_drones, dt, component, v_max)
            % Define colors for the plot
            green = [0, 128, 0] / 255;    % Green color (RGB: 0,128,0)
            blue = [0, 0, 205] / 255;     % Blue for real data
            red = [139, 0, 0] / 255;      % Red color (RGB: 139,0,0)
            orange = [0.922, 0.38, 0.24]; % Orange color (hex: #eb613d)
        
            % Determine the index of the selected component and its labels
            switch component
                case 'velocity-x'
                    component_idx = 1;  
                    label = 'velocity-x';
                    ylabel_text = '$v_x$ [m/s]';
                    show_vmax = true;
                case 'velocity-y'
                    component_idx = 2; 
                    label = 'velocity-y';
                    ylabel_text = '$v_y$ [m/s]';
                    show_vmax = true;
                case 'position-x'
                    component_idx = 1; 
                    label = 'position-x';
                    ylabel_text = '$x$ [m]';
                    show_vmax = false;
                case 'position-y'
                    component_idx = 2; 
                    label = 'position-y';
                    ylabel_text = '$y$ [m]';
                    show_vmax = false;
                otherwise
                    error('Invalid component selected');
            end
        
            % Create figure with Full HD dimensions
            figure('Position', [100, 100, 1920, 1080]); % Full HD dimensions
        
            % Set up tiled layout
            tiledlayout(n_drones, 1, 'TileSpacing', 'Compact', 'Padding', 'Compact');
        
            % Initialize handles for legend
            legend_handles = [];
            xlabel_handles = gobjects(n_drones, 1); % Preallocate for xlabel handles
        
            % Loop through each drone to create subplots
            for i = 1:n_drones
                % Extract input and real data for this drone
                input_values = squeeze(input_data(i, :, component_idx)); 
                real_values = squeeze(real_data(i, :, component_idx));   
                input_values = input_values(~isnan(input_values));
                real_values = real_values(~isnan(real_values));
        
                % Actual simulation time based on data collected
                sim_time_real = (length(real_values) - 1) * dt; % Real data end time
                sim_time_input = (length(input_values) - 1);    % Input data end time

                % Use the smaller of the two
                actual_sim_time = min(sim_time_real, sim_time_input);

                % Create aligned time vectors
                time_real = 0:dt:actual_sim_time; % Real data time points
                time_input = 0:1:actual_sim_time; % Input data time points

                %fprintf('Time Input Range: [%.2f, %.2f]\n', time_input(1), time_input(end));
                %fprintf('Time Real Range: [%.2f, %.2f]\n', time_real(1), time_real(end));

                % Truncate input and real data to match actual_sim_time
                max_real_idx = min(length(real_values), length(time_real));
                max_input_idx = min(length(input_values), length(time_input));

                % Truncate time vectors to match the truncated data
                time_real = time_real(1:max_real_idx);
                time_input = time_input(1:max_input_idx);

                % Truncate data arrays
                real_values = real_values(1:max_real_idx);
                input_values = input_values(1:max_input_idx);

                % Calculate replication factor
                replication_factor = round(length(time_real) / length(time_input));

                % Use the replication factor for repelem
                expanded_input = repelem(input_values, replication_factor);

                % Ensure expanded_input and real_values have the same length
                common_length = min(length(expanded_input), length(real_values));
                expanded_input = expanded_input(1:common_length);
                real_values = real_values(1:common_length);
                time_real = time_real(1:common_length);
                % Create subplot for each drone
                nexttile;
                hold on;
                % Plot input data as step functions
                h1 = stairs(time_real, expanded_input, 'Color', green, 'LineWidth', 3, 'DisplayName', ['Input ', label]);
                % Plot real data as continuous curves
                h2 = plot(time_real, real_values, 'Color', blue, 'LineWidth', 1.5, 'DisplayName', ['Real ', label]);
                % Add horizontal lines for v_max and -v_max if applicable
                if show_vmax
                    h3 = yline(v_max, 'Color', red, 'LineWidth', 1.5); % v_max
                    h4 = yline(-v_max, 'Color', orange, 'LineWidth', 1.5); % -v_max
                end
                hold off;
        
                % Collect handles only for the first subplot
                if i == 1
                    if show_vmax
                        legend_handles = [h1, h2, h3, h4];
                    else
                        legend_handles = [h1, h2];
                    end
                end
        
                % Set axis labels and title
                xlabel_handles(i) = xlabel('$t$ [s]', 'Interpreter', 'latex', 'FontSize', obj.font_labels - 8);
                ylabel(ylabel_text, 'Interpreter', 'latex', 'FontSize', obj.font_labels - 8);
                title(sprintf('Drone %d', i), 'Interpreter', 'latex', 'FontSize', obj.font_title - 5);
        
                % Add grid
                grid on;
            end
        
            % Adjust x-label positions after all plots are created
            for i = 1:n_drones
                x_limits = xlim; % Get x-axis limits for the subplot
                xlabel_handles(i).Position(1) = x_limits(2) + 2; % Shift the label slightly beyond the right x-limit
                xlabel_handles(i).HorizontalAlignment = 'right'; % Align the label to the right
            end
        
            % Add a global title
            sgtitle(['Input vs Real ', label], 'Interpreter', 'latex', 'FontSize', obj.font_title - 3);
        
            % Add a single legend outside the plotting area
            if show_vmax
                legend_labels = {'Input', 'Real', '$v_{\mathrm{max}}$', '$-v_{\mathrm{max}}$'}; 
            else
                legend_labels = {'Input', 'Real'}; 
            end
            lg = legend(legend_handles, legend_labels, ...
                'Interpreter', 'latex', 'FontSize', obj.font_legend, 'Orientation', 'horizontal');
            lg.Layout.Tile = 'north'; % Place the legend above the subplots
        
            % Add the annotation box
            a = annotation('rectangle', [0 0 1 1], 'Color', 'w');
        
            % Export the figure
            exportgraphics(gcf, fullfile('figures', ['comparison_', label, '_pso.pdf']), 'ContentType', 'vector');
        
            % Remove the annotation box
            delete(a);
        end        
    end        
end

