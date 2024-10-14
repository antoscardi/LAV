classdef TotalMagneticField
    properties
        vector_fields                  % Cell array to store 3x1 vectors of individual magnetic field vectors
    end
    
    methods
        % Constructor
        function obj = TotalMagneticField()
            obj.vector_fields = {}; 
        end
        
        % Method to compute a single magnetic field and store it
        function obj = magnetic_field(obj, point)
            if size(point, 1) ~= 3 
                error('Point must be a 3x1 vector.');
            end
            
            % Calculate the magnetic field vectors for the current point
            field_vector = obj.compute_field_at_point(point);
            
            % Store the computed magnetic field
            obj.vector_fields{end + 1} = field_vector;
        end
        
        % Method to compute the sum of all stored magnetic fields
        function total_field = sum_fields(obj)
            if isempty(obj.vector_fields)
                error('No magnetic fields stored to compute the sum.');
            end
            % Initialize total_field with zeros
            total_field = zeros(3, 1);
            
            % Sum all stored magnetic fields
            for i = 1:length(obj.vector_fields)
                total_field = total_field + obj.vector_fields{i};
            end
        end
        
       % Method to compute the NSS at a given point using the Jacobian
        function NSS = NormalizedSourceStrength(obj, point)
            % Calculate the total field at the given point
            total_field = obj.sum_fields();

            % Initialize the Jacobian matrix
            J = zeros(3, 3);
            delta = 1e-6; % Small perturbation for numerical differentiation

            % Loop over each direction (x, y, z) for numerical differentiation
            for k = 1:3
                perturbation = zeros(3, 1);
                perturbation(k) = delta;

                % Compute the total field at the perturbed points
                perturbed_field_plus = obj.compute_total_field_at_point(point + perturbation);
                perturbed_field_minus = obj.compute_total_field_at_point(point - perturbation);

                % Approximate the derivative as (f(x+delta) - f(x-delta)) / (2*delta)
                J(:, k) = (perturbed_field_plus - perturbed_field_minus) / (2 * delta);

                % Debug statements to check the computation
                disp(['Direction ', num2str(k)]);
                disp('Perturbation:');
                disp(perturbation);
                disp('Perturbed Field Plus:');
                disp(perturbed_field_plus);
                disp('Perturbed Field Minus:');
                disp(perturbed_field_minus);
                disp('Derivative:');
                disp(J(:, k));
            end

            % Compute eigenvalues of the Jacobian matrix
            eigenvalues = eig(J);

            % Sort eigenvalues in ascending order
            sorted_eigenvalues = sort(eigenvalues);

            % Extract the sorted eigenvalues
            eigen_min = sorted_eigenvalues(1);
            eigen_middle = sorted_eigenvalues(2);
            eigen_max = sorted_eigenvalues(3);

            % Compute the NSS using the formula
            NSS = sqrt(-eigen_middle^2 - eigen_max * eigen_min);
            NSS = real(NSS); % Ensure NSS is a real number

            % Debug statement for eigenvalues and NSS
            disp('Eigenvalues:');
            disp(sorted_eigenvalues);
            disp('NSS:');
            disp(NSS);
        end


        % Helper function to compute the total magnetic field at a given point
        function total_field = compute_total_field_at_point(obj, point)
            % Initialize total_field with zeros
            total_field = zeros(3, 1);
            % Recompute the fields for each stored vector at the new point
            for i = 1:length(obj.vector_fields)
                % Compute the magnetic field for each source at the perturbed point
                field_vector = obj.compute_field_at_point(point);
                % Sum it into the total field
                total_field = total_field + field_vector;
            end
        end

        % Helper function to compute the magnetic field at a given point
        function field = compute_field_at_point(~, point)
            % Calculate the magnetic field vectors for the given point
            x = point(1);
            y = point(2);
            z = point(3);
            
            r = norm(point);
            r(r == 0) = 1e-11;  % Avoid division by zero
            
            Hx = (3 * x * z) / r^5;
            Hy = (3 * y * x) / r^5;
            Hz = (3 * z^2 / r^2 - 1) / r^3;
            
            field = [Hx; Hy; Hz];
        end
    end
end
