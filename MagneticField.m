classdef MagneticField
    properties
        point                           % 2xN or 3xN matrix of points where the field is computed
        vector_field                    % 2xN or 3xN matrix of magnetic field vectors 
        I = 10;                         % Current (default: 10 A)
        mu = 4 * pi * 1e-7;             % Magnetic constant (default: permeability of free space)
        b = 0.1;                        % Coil radius (default: 10 cm)
        increasing_factor = 1000        % In order to have better calculations we need to amplify the mag field
        visualization_scaling = 10^2;   % Visualization scaling factor
        prefactor
    end
    
    methods
        % Constructor
        function obj = MagneticField()
            obj.point = [];        
            obj.vector_field = []; 
        end
        
        % Method to set points and calculate the magnetic field
        function obj = compute(obj, point, varargin)
            if size(point, 1) ~= 3 
                error('Points must be a 3xN');
            end
            % Update points property
            obj.point = point;
            % Initialize field_vectors to zeros with appropriate dimensions
            obj.vector_field = zeros(size(point));
            
            % Calculate the magnetic field vectors for the current points
            x = obj.point(1, :);
            y = obj.point(2, :);
            z = obj.point(3, :);
            
            r =  sqrt(x.^2 + y.^2 + z.^2); % Magnitude of vector
            % Avoid division by zero by replacing zeros with a small number
            r(r == 0) = eps;
            
            % Magnetic field calculation 
            obj.prefactor = (obj.I * obj.b^2 * obj.increasing_factor) ./ (4 * r.^5);
            Hx = obj.prefactor .* (3*x.*z);
            Hy = obj.prefactor .* (3*y.*z);
            Hz = obj.prefactor .* (2*z.^2 - x.^2 - y.^2);
            obj.vector_field = [Hx;Hy;Hz];
        end
    
        % Method to plot the magnetic field in a given Space object
        function plot(obj, spaceObj, n_sources, i, visualization_2D)
            if isempty(obj.point) || isempty(obj.vector_field)
                error('No points or field vectors defined. Please calculate the field using calculateFieldAtPoints method first.');
            end
            if ~isa(spaceObj, 'Space')
                error('Input must be a Space object.');
            end
            % Colors
            C = linspecer(n_sources);   
            if ~visualization_2D
                % Scale vectors only for visualization
                scaled_field_vectors = obj.visualization_scaling * obj.vector_field;
                % Plot the magnetic field vectors using the space's axes for 3D
                quiver3(spaceObj.Axes, spaceObj.points(1, :), spaceObj.points(2, :), spaceObj.points(3, :), ...
                        scaled_field_vectors(1, :), scaled_field_vectors(2, :), scaled_field_vectors(3, :), ...
                        'k', 'LineWidth', 2, 'DisplayName', ['Magnetic Field n', num2str(i)], 'Color', C(i, :), ...
                        'Autoscale', 'off', 'MaxHeadSize', 0.2);
            elseif visualization_2D
                % Scale vectors only for visualization
                scaled_field_vectors = obj.visualization_scaling * obj.vector_field;
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