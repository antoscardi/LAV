classdef Space
    properties
        Name
        Axes         % Axes handle for the plot
        Dimension    % Dimension of the space (2 or 3)
        e1           % Unit vector along the X-axis
        e2           % Unit vector along the Y-axis
        e3           % Unit vector along the Z-axis (only for 3D)
        points       % plot points in space
        mesh_x
        mesh_y
        mesh_z
    end
    
    methods
        % Constructor
        function obj = Space(name, dimension, n_points)
            % Set the dimension (default to 3D if not specified)
            if nargin < 2
                dimension = 3;
            end
            
            % Validate the dimension input
            if dimension ~= 2 && dimension ~= 3
                error('Dimension must be either 2 or 3.');
            end
            
            obj.Name = name;
            obj.Dimension = dimension;

            x = linspace(-100, 100, n_points);
            y = linspace(-100, 100, n_points);
            z = linspace(0, 100, n_points);
            [x, y, z] = meshgrid(x,y,z);
            obj.mesh_x = x;
            obj.mesh_y = y;
            obj.mesh_z = z;
            obj.points = [x(:)'; y(:)'; z(:)']; % Reshape the grid into a 3xN matrix of points

            % Initialize unit vectors and points
            if obj.Dimension == 3
                obj.e1 = [1; 0; 0]; % X-axis unit vector
                obj.e2 = [0; 1; 0]; % Y-axis unit vector
                obj.e3 = [0; 0; 1]; % Z-axis unit vector
            elseif obj.Dimension == 2
                obj.e1 = [1; 0];    % X-axis unit vector
                obj.e2 = [0; 1];    % Y-axis unit vector
                obj.e3 = [];        % No Z-axis unit vector in 2D
                % Project the points in 2D
                obj.points = obj.points(1:2, :);
            end

            figure; 
            obj.Axes = axes('XGrid', 'on', 'YGrid', 'on');
            if obj.Dimension == 3
                axis vis3d;       % Freeze aspect ratio properties to enable rotation
                view(3);          % Set the default 3D view
                daspect([1 1 1]); % Set data aspect ratio for equal scaling
                obj.Axes.ZGrid = 'on';
                zlabel('z');
            end
            axis equal; % Equal scaling on all axes
            xlabel('x');
            ylabel('y');
            title(obj.Name);
            hold on;

            if obj.Dimension == 3
                obj.plotCoordinateSystem(eye(3), [0, 0, 0], 'Inertial', 40);
                % Plot the points in the space
                plot3(obj.Axes, obj.points(1, :), obj.points(2, :), obj.points(3, :), ...
                    'o', 'MarkerSize', 0.1, 'DisplayName', 'points', 'Color', [0 0 0]);
            elseif obj.Dimension == 2
                obj.plotCoordinateSystem(eye(2), [0, 0], 'Inertial', 40);
                % Plot the points in the space
                plot(obj.Axes, obj.points(1, :), obj.points(2, :), ...
                    'o', 'MarkerSize', 0.1, 'DisplayName', 'points', 'Color', [0 0 0]);
            end
            legend(obj.Axes, 'show', 'Location', 'northeastoutside');      
        end
    
        % Method to plot a coordinate system with given rotation matrix and origin
        function plotCoordinateSystem(obj, R, origin, name, do_projection, scaling)
            % Handle default values for do_projection and scaling
            if nargin < 5
                do_projection = false;  % Default value for do_projection
                scaling = 30;           % Default value for scaling
            elseif nargin < 6
                if islogical(do_projection)
                    scaling = 30;       % If only do_projection is specified, set scaling to default
                else
                    scaling = do_projection; % The fifth argument was meant to be scaling
                    do_projection = false;   % Set do_projection to default
                end
            end

            if obj.Dimension == 3
                x_axis = R * obj.e1 * scaling;
                y_axis = R * obj.e2 * scaling;
                z_axis = R * obj.e3 * scaling;
                % Plot the X, Y, Z axes using quiver3 for 3D
                quiver3(obj.Axes, origin(1), origin(2), origin(3), x_axis(1), x_axis(2), x_axis(3), ...
                    'r', 'LineWidth', 2, 'DisplayName', [name, ' x'], 'AutoScale', 'off');
                quiver3(obj.Axes, origin(1), origin(2), origin(3), y_axis(1), y_axis(2), y_axis(3), ...
                    'g', 'LineWidth', 2, 'DisplayName', [name, ' y'], 'AutoScale', 'off');
                quiver3(obj.Axes, origin(1), origin(2), origin(3), z_axis(1), z_axis(2), z_axis(3), ...
                    'b', 'LineWidth', 2, 'DisplayName', [name, ' z'], 'AutoScale', 'off');

            elseif obj.Dimension == 2
                if do_projection
                    % Using 3D unit vectors for projection in a 2D plot
                    x_axis = R * [1; 0; 0] * scaling;
                    y_axis = R * [0; 1; 0] * scaling;
                    x_axis = x_axis(1:2); % Project to 2D
                    y_axis = y_axis(1:2); % Project to 2D
                else
                    x_axis = R * obj.e1 * scaling;
                    y_axis = R * obj.e2 * scaling;
                end
                % Plot the X, Y axes using quiver
                quiver(obj.Axes, origin(1), origin(2), x_axis(1), x_axis(2), ...
                    'r', 'LineWidth', 2, 'DisplayName', [name, ' x'], 'AutoScale', 'off'); 
                quiver(obj.Axes, origin(1), origin(2), y_axis(1), y_axis(2), ...
                    'g', 'LineWidth', 2, 'DisplayName', [name, ' y'], 'AutoScale', 'off');
            end
        end

        % Method to generate a random rotation matrix
        function R = RotationMatrix(obj, i, varargin)
            % Initialize angles to zero
            roll = 0;
            pitch = 0;
            yaw = 0;
            theta = 0;

            % Check the dimension and assign input values to angles
            if obj.Dimension == 3
                if length(varargin) >= 1
                    roll = varargin{1}; % Set roll to the first input, or remain 0
                end
                if length(varargin) >= 2
                    pitch = varargin{2}; % Set pitch to the second input, or remain 0
                end
                if length(varargin) >= 3
                    yaw = varargin{3}; % Set yaw to the third input, or remain 0
                end

                Rx = [1, 0, 0;
                      0, cos(yaw), -sin(yaw);
                      0, sin(yaw), cos(yaw)];

                Ry = [cos(pitch), 0, sin(pitch);
                      0, 1, 0;
                      -sin(pitch), 0, cos(pitch)];

                Rz = [cos(roll), -sin(roll), 0;
                      sin(roll), cos(roll), 0;
                      0, 0, 1];

                R = Rz * Ry * Rx;

            elseif obj.Dimension == 2
                % Expecting a single angle for 2D rotation
                if ~isempty(varargin)
                    theta = varargin{1}; % Set theta to the first input, or remain 0
                end

                R = [cos(theta), -sin(theta);
                     sin(theta), cos(theta)];         
            end

            %fprintf('Rotation Matrix of frame %s:\n', num2str(i));
            %disp(R);
        end
    end
end
