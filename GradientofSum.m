classdef GradientofSum
    properties
        magnetic_fields    % Cell array to store MagneticField objects
        field_vectors_sum  % 3xN matrix of the summed magnetic field vectors
        points             % 3xN matrix of the points
        x
        y
        z
        num_points         % Number of points
        dimension          % Dimension of the space (3 for 3D)
        G                  % Gradient matrix, 3x3xN tensor
        N                  % matrix dimension
        I                  % inclination of the magnetic field wrt the xy plane
    end
    
    methods
        % Constructor to initialize GradientofSum object
        function obj = GradientofSum(magnetic_fields, spaceObj)
            % Validate input
            if ~iscell(magnetic_fields) || isempty(magnetic_fields)
                error('Input must be a non-empty cell array of MagneticField objects.');
            end
            if ~isa(spaceObj, 'Space')
                error('spaceObj must be an instance of the Space class.');
            end
            for k = 1:length(magnetic_fields)
                if ~isa(magnetic_fields{k}, 'MagneticField')
                    error('Each element in magnetic_fields must be a MagneticField object.');
                end
            end
            
            % Store the magnetic fields, points, and dimension
            obj.magnetic_fields = magnetic_fields;
            obj.points = spaceObj.points;
            obj.x = unique(spaceObj.mesh_x);
            obj.y = unique(spaceObj.mesh_y);
            obj.z = unique(spaceObj.mesh_z);
            obj.num_points = size(obj.points, 2);
            obj.dimension = size(obj.points, 1); % Set dimension (3 for 3D)
            
            % Initialize the field_vectors_sum and gradient to empty
            obj.field_vectors_sum = [];
            obj.G = zeros(obj.dimension, obj.dimension, obj.num_points);
        end
        
        % Method to sum the magnetic field vectors
        function obj = sumFieldVectors(obj)
            % Initialize the sum of field vectors to zeros
            obj.field_vectors_sum = zeros(obj.dimension, obj.num_points);
            
            % Loop over each MagneticField object and sum their vectors
            for k = 1:length(obj.magnetic_fields)
                obj.field_vectors_sum = obj.field_vectors_sum + obj.magnetic_fields{k}.vector_field;
            end

            %obj.field_vectors_sum = obj.field_vectors_sum * 10^6;
            
            I_vect = atan2(abs(obj.field_vectors_sum(3, :)), ...
                    sqrt(obj.field_vectors_sum(1, :).^2 + obj.field_vectors_sum(2, :).^2));
            %obj.I = rad2deg(mode(I_vect)); % valore piu comune quindi quello dei singoli campi
            %obj.I = rad2deg(mean(I_vect)); % valore medio
            obj.I = reshape(rad2deg(I_vect),[], 1); % ad ogni punto il suo valore
            disp(['Inclination Angle (degrees): ', num2str(size(obj.I))]);
        end
        
        function obj = computeGradient(obj)
            % Check if field_vectors_sum is empty
            if isempty(obj.field_vectors_sum)
                error('No sum, field_vectors_sum is empty. Please call sumFieldVectors first.');
            end
            
            % Check if there are at least two points to compute gradient
            if obj.num_points < 2
                error('Not enough points to compute gradient. At least two points are required.');
            end
            
            % Assuming obj.x, obj.y, obj.z are the meshgrid coordinates
            size_x = size(obj.x);
            size_y = size(obj.y);
            size_z = size(obj.z);
            obj.N = size_x(1) * size_y(1) * size_z(1);            % Total number of points expected
        
            % Check if the number of points matches
            if size(obj.field_vectors_sum, 2) ~= obj.N
                disp(size(obj.field_vectors_sum, 2))
                error('Number of points in field_vectors_sum does not match the number of grid points.');
            end
            
            % Extract spacing
            dx = obj.x(2) - obj.x(1);
            dy = obj.y(2) - obj.y(1);
            dz = obj.z(2) - obj.z(1);
            
            % Reshape field vectors to match the 3D grid using []
            Hx = reshape(obj.field_vectors_sum(1, :), [size_x(1), size_y(1), size_z(1)]);
            Hy = reshape(obj.field_vectors_sum(2, :), [size_x(1), size_y(1), size_z(1)]);
            Hz = reshape(obj.field_vectors_sum(3, :), [size_x(1), size_y(1), size_z(1)]);
            
            % Compute gradients specifying the correct spacing for each axis
            [gradVx_X, gradVx_Y, gradVx_Z] = gradient(Hx, dx, dy, dz);
            [gradVy_X, gradVy_Y, gradVy_Z] = gradient(Hy, dx, dy, dz);
            [gradVz_X, gradVz_Y, gradVz_Z] = gradient(Hz, dx, dy, dz);
            
            % Combine into gradient tensor (Jacobian matrix)
            obj.G = zeros(3, 3, obj.N);  % Initialize gradient matrix
            for i = 1:size_x(1)
                for j = 1:size_y(1)
                    for k = 1:size_z(1)
                        idx = sub2ind([size_x(1), size_y(1), size_z(1)], i, j, k);  % Linear index
                        obj.G(:, :, idx) = ...
                            [gradVx_X(i, j, k), gradVx_Y(i, j, k), gradVx_Z(i, j, k);
                             gradVy_X(i, j, k), gradVy_Y(i, j, k), gradVy_Z(i, j, k);
                             gradVz_X(i, j, k), gradVz_Y(i, j, k), gradVz_Z(i, j, k)];
                    end
                end
            end
            
            %{
            obj.G = zeros(obj.dimension, obj.dimension, obj.num_points); 
            for k = 1:length(obj.magnetic_fields)
                obj.G = obj.G + obj.magnetic_fields{k}.G;
            end
            %}
            
        end

        function checkSymmetry(obj)
            % Method to check if the gradient tensor G is symmetric at all points
            numAsymmetric = 0; % Counter for asymmetric matrices
            tolerance = 1e-6;  % Tolerance for numerical precision issues
        
            for idx = 1:obj.N
                G_matrix = obj.G(:, :, idx);
                % Check if the gradient matrix is symmetric
                if max(max(abs(G_matrix - G_matrix'))) > tolerance
                    numAsymmetric = numAsymmetric + 1;
                    fprintf('Matrix at point %d is not symmetric for %d.\n', idx, max(max(abs(G_matrix - G_matrix'))));
                end
            end
        
            if numAsymmetric == 0
                disp('All gradient matrices are symmetric.');
            else
                fprintf('%d of %d gradient matrices are not symmetric.\n', numAsymmetric, obj.N);
            end
        end
                
          
        % Method to compute THDz
        function THDz = computeTHDz(obj)
            Bxz = squeeze(obj.G(1, 3, :));
            Byz = squeeze(obj.G(2, 3, :));
            THDz = sqrt(Bxz.^2 + Byz.^2);
        end

        % Method to compute Analytic Signal (AS)
        function AS = computeAS(obj)
            Bxz = squeeze(obj.G(1, 3, :));
            Byz = squeeze(obj.G(2, 3, :));
            Bzz = squeeze(obj.G(3, 3, :));
            AS = sqrt(Bxz.^2 + Byz.^2 + Bzz.^2);
        end

        % Method to compute Tilt angle (Tilt)
        function Tilt1 = computeTilt1(obj)
            Bzz = squeeze(obj.G(3, 3, :));
            Bxx = squeeze(obj.G(1, 1, :));
            Byy = squeeze(obj.G(2, 2, :));

            Tilt1 = atan2((sign(obj.I).*Bzz), sqrt(Bxx.^2 + Byy.^2));
        end

        % Method to compute Theta map (Theta)
        function Theta = computeTheta(obj)
            Bxz = squeeze(obj.G(1, 3, :));
            Byz = squeeze(obj.G(2, 3, :));
            Bzz = squeeze(obj.G(3, 3, :));
            
            disp(['Bxz sample values: ', num2str(Bxz(1:5)')]);
            disp(['Byz sample values: ', num2str(Byz(1:5)')]);
            disp(['Bzz sample values: ', num2str(Bzz(1:5)')]);
            
            denominator = sqrt(Bxz.^2 + Byz.^2 + Bzz.^2);
            numerator = sqrt(Bxz.^2 + Byz.^2);
            
            Theta = acos(numerator ./ denominator);
            disp(['Theta sample values: ', num2str(Theta(1:5)')]);
        end

        function Tilt2 = computeTilt2(obj)
            Bzz = squeeze(obj.G(3, 3, :));
            Bxz = squeeze(obj.G(1, 3, :));
            Byz = squeeze(obj.G(2, 3, :));

            Tilt2 = atan2((sign(obj.I).*Bzz), sqrt(Bxz.^2 + Byz.^2));
        end

        function NSS = computeNSS(obj)
            % Inputs:
            %    G - G 3x3xN matrix where each 3x3 slice represents the gradient tensor.
            % Outputs:
            %    NSS - A 1xN array containing the NSS values for each 3x3 matrix.
            
            % Preallocate array to store NSS values
            NSS = zeros(1, obj.N);
        
            % Loop through each matrix and calculate the eigenvalues and NSS
            for i = 1:obj.N
                % Extract the i-th 3x3 matrix
                gradient_i = obj.G(:, :, i);
                
                % Compute eigenvalues
                lambdas = eig(gradient_i);
                
                % Sort eigenvalues in ascending order (if not already sorted)
                lambdas = sort(lambdas);
            
                lambda1 = lambdas(1);
                lambda2 = lambdas(2);
                lambda3 = lambdas(3);
                
                scaling_factor = 10^7;
                % Compute NSS using the given formula
                NSS(i) = scaling_factor *sqrt(-lambda2^2 - lambda1 * lambda3);
            end
        end        
     
        function values = plotResults(obj, methodName)
            % Determine the method to compute values
            switch methodName
                case 'THDz'
                    values = obj.computeTHDz();
                case 'AS'
                    values = obj.computeAS();
                case 'Tilt1'
                    values = obj.computeTilt1();
                case 'Theta'
                    values = obj.computeTheta();
                case 'Tilt2'
                    values = obj.computeTilt2();
                case 'NSS'
                    values = obj.computeNSS();
                otherwise
                    error('Unknown method name');
            end
        
            % Identify the points on the z = 0 plane
            z_zero_indices = obj.points(3, :) == 0;
            x_plane = obj.points(1, z_zero_indices); % x-coordinates on the z=0 plane
            y_plane = obj.points(2, z_zero_indices); % y-coordinates on the z=0 plane
            values_plane = values(z_zero_indices);   % Values corresponding to z=0 plane
        
            % Plot the values directly as a scatter plot
            scatter(x_plane, y_plane, 40, values_plane, 'filled');
            colorbar; % Display the color scale
    
            % Define common title and labels
            title([methodName ' Edge Detection on z=0 Plane']);
            xlabel('X-axis');
            ylabel('Y-axis');
        
            % Update figure
            drawnow; % Ensure all graphics operations are completed
        end
    end        
end




