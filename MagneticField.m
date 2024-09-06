classdef MagneticField
    properties
        points        % 2xN or 3xN matrix of points where the field is computed
        field_vectors % 2xN or 3xN matrix of magnetic field vectors 
        I = 10;        % Current (default: 10 A)
        mu = 4 * pi * 1e-7;  % Magnetic constant (default: permeability of free space)
        b = 0.1;      % Coil radius (default: 10 cm)
        visualization_scaling = 10^5; % Visualization scaling factor
        prefactor
        G
    end
    
    methods
        % Constructor - initializes an empty MagneticField object
        function obj = MagneticField()
            obj.points = [];        % Initialize points as empty
            obj.field_vectors = []; % Initialize field_vectors as empty
        end
        
        % Method to set points and calculate the magnetic field
        function obj = calculateFieldAtPoints(obj, points, spaceObj)
            % Validate points
            if size(points, 1) ~= 3 && size(points, 1) ~= 2
                error('Points must be a 3xN or 2xN matrix.');
            end
            
            % Update points property
            obj.points = points;
            
            % Initialize field_vectors to zeros with appropriate dimensions
            obj.field_vectors = zeros(size(points));
            
            % Calculate the magnetic field vectors for the current points
            x = obj.points(1, :);
            y = obj.points(2, :);
            z = obj.points(3, :);
            
            r_squared = x.^2 + y.^2 + z.^2;
            r_mag = sqrt(r_squared);  % Magnitude of vector
            
            % Avoid division by zero by replacing zeros with a small number
            r_mag(r_mag == 0) = eps;
            
            % Magnetic field calculation 
            obj.prefactor = (obj.I * obj.b^2) ./ (4 * r_mag.^5);
            Hx = obj.prefactor .* (3*x.*z);
            Hy = obj.prefactor .* (3*y.*z);
            Hz = obj.prefactor .* (2*z.^2 - x.^2 - y.^2);
            
            % Set field_vectors
            if spaceObj.Dimension == 3
                obj.field_vectors = [Hx; Hy; Hz];
            elseif spaceObj.Dimension == 2
                % Project the points and the field vectors
                obj.field_vectors = [Hx; Hy];
                obj.points = obj.points(1:2, :);
            end
        end
        
        function obj = gradient_tensor(obj)
            % Extract coordinates
            x = obj.points(1, :);
            y = obj.points(2, :);
            z = obj.points(3, :);
            num_points = length(x);  % Assuming x, y, z have the same length

            % Initialize the gradient tensor
            obj.G = zeros(3, 3, num_points);

            % Calculate the gradient tensor for each point
            for i = 1:num_points
                % Compute the gradient matrix at each point
                G_i = obj.prefactor(i) * [
                    3 * z(i), 0, 3 * x(i);
                    0, 3 * z(i), 3 * y(i);
                    -2 * x(i), -2 * y(i), 4 * z(i)
                ];
                % Assign the gradient matrix to the tensor
                obj.G(:, :, i) = G_i;
            end
        end
    
        % Method to plot the magnetic field in a given Space object
        function plotFieldInSpace(obj, spaceObj, n_sources, i)
            if isempty(obj.points) || isempty(obj.field_vectors)
                error('No points or field vectors defined. Please calculate the field using calculateFieldAtPoints method first.');
            end
            
            if ~isa(spaceObj, 'Space')
                error('Input must be a Space object.');
            end
            % Colors
            C = linspecer(n_sources);
            
            if spaceObj.Dimension == 3 && size(spaceObj.points, 1) == 3
                % Scale vectors only for visualization
                scaled_field_vectors = obj.visualization_scaling * obj.field_vectors;
                % Plot the magnetic field vectors using the space's axes for 3D
                quiver3(spaceObj.Axes, spaceObj.points(1, :), spaceObj.points(2, :), spaceObj.points(3, :), ...
                        scaled_field_vectors(1, :), scaled_field_vectors(2, :), scaled_field_vectors(3, :), ...
                        'k', 'LineWidth', 2, 'DisplayName', ['Magnetic Field n', num2str(i)], 'Color', C(i, :), ...
                        'Autoscale', 'off', 'MaxHeadSize', 0.2);
            elseif spaceObj.Dimension == 2 && size(spaceObj.points, 1) == 2
                % Scale vectors only for visualization
                scaled_field_vectors = obj.visualization_scaling * obj.field_vectors;
                % Plot the magnetic field vectors using the space's axes for 2D
                quiver(spaceObj.Axes, spaceObj.points(1, :), spaceObj.points(2, :), ...
                    scaled_field_vectors(1, :), scaled_field_vectors(2, :), ...
                    'k', 'LineWidth', 2, 'DisplayName', ['Magnetic Field n', num2str(i)], 'Color', C(i, :), ...
                    'Autoscale', 'off', 'MaxHeadSize', 0.2);
            else
                error('Dimension mismatch between MagneticField object and Space object.');
            end
        end
    end
end

