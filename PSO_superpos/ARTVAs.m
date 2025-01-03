%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                   ARTVAs CLASS                                                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The ARTVAs class handles the calculation of NSS (Normalized Source Strength) signals from multiple sources. It 
% provides methods to compute the combined NSS value from all sources and individual NSS values based on the 
% distance between a drone and a source.
%
% Methods:
% - superpositionNSS: Calculates the total NSS received by a drone from all sources.
% - send_signal_NSS: Calculates the NSS (signal strength) from a specific source based on distance.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef ARTVAs
    properties
        Ib = 10000000;  % It was 100 before
        R_array;
        R_drones;
    end
    
    methods
        % Constructor to initialize scaling factor C
        function obj = ARTVAs(n_drones, n_sources)
            % Initialize rotation matrices to identity
            obj.R_array = cell(n_sources, 1);
            obj.R_drones = cell(n_drones, 1);
            for i = 1:n_sources
                obj.R_array{i} = eye(3);
            end
            for i = 1:n_drones
                obj.R_drones{i} = eye(3);
            end
        end

        %Superposition NSS Function: Sum of the NSS from all sources
        function value_tot_new = superpositionNSS(obj, position, p_sources)
             value_tot_old = 0;
             value_tot_new = 0;
             J = zeros(3,3);
             % PUT THE POSITION 
             position = [position, 3];
             p_sources = [p_sources, 0 * ones(size(p_sources, 1), 1)];
             for i = 1:size(p_sources, 1)
                % METHOD NEW COMPLETE
                J = J + obj.R_drones{i} * obj.send_signal_NSS_new(position, p_sources(i, :), i) * obj.R_drones{i}.';
                value_tot_new = obj.eigenvalues(J);
                % METHOD OLD SIMPLIFIED
                value_tot_old = value_tot_old + obj.send_signal_NSS_old(position, p_sources(i, :));
             end
             %fprintf('NSS SIMPL: %.1f, NSS TRUE: %.1f\n', value_tot_old, value_tot_new);  % Debug print
             return;
         end

        % Function to compute the Jacobian matrix analytically
        function J = send_signal_NSS_new(obj, point, p_source, i)
            % Compute the local point relative to the source
            distance_vector = point - p_source;
            local_point = obj.R_array{i}' * distance_vector';
            
            x = local_point(1);
            y = local_point(2);
            z = local_point(3);
            denominator = (x^2 + y^2 + z^2)^(7/2) / obj.Ib;

            dHx_dx = 3 * z * (-4 * x^2 + y^2 + z^2) / denominator;
            dHx_dy = -15 * x * y * z / denominator;
            dHx_dz = 3 * x * (x^2 + y^2 - 4 * z^2) / denominator;

            dHy_dx = -15 * x * y * z / denominator;
            dHy_dy = 3 * z * (x^2 - 4 * y^2 + z^2) / denominator; 
            dHy_dz = 3 * y * (x^2 + y^2 - 4 * z^2) / denominator;
            
            dHz_dx = 3 * x * (x^2 + y^2 - 4 * z^2) / denominator;
            dHz_dy = 3 * y * (x^2 + y^2 - 4 * z^2) / denominator;
            dHz_dz = 3 * z * (3 * x^2 + 3 * y^2 - 2 * z^2) / denominator;
            
            % Assemble the Jacobian for this source's contribution
            J = [dHx_dx, dHy_dx, dHz_dx;
                        dHx_dy, dHy_dy, dHz_dy;
                        dHx_dz, dHy_dz, dHz_dz];   
        end

        function NSS_value = eigenvalues(obj, J)

            % Compute eigenvalues of the Jacobian matrix
            eigenvalues = eig(J);
        
            % Sort eigenvalues in ascending order
            sorted_eigenvalues = sort(eigenvalues);
        
            % Extract the sorted eigenvalues
            eigen_min = sorted_eigenvalues(1);
            eigen_middle = sorted_eigenvalues(2);
            eigen_max = sorted_eigenvalues(3);
        
            % Compute the NSS using the formula
            NSS_value = sqrt(-eigen_middle^2 - eigen_max * eigen_min);
            NSS_value = real(NSS_value);
        end
        
        
        % NSS Function: Normalized Source Strength (Signal Function) OLDER FUNCTION WITH SIMPLIFIED CALCULATIONS
         function NSS_value = send_signal_NSS_old(obj, position, p_source)
             % Calculate the distance between the drone and the source
             r = sqrt(sum((position - p_source).^2));  % Use element-wise power
            
             % Prevent division by zero by setting a minimum distance
             r = max(r, 1e-10);
            
             % Calculate normalized source strength (NSS) based on distance
             NSS_value = 3 * obj.Ib / r^4;  % Signal strength diminishes rapidly with distance
             % We put the 3 to make it equal to the eigenvalues
         end
    end
end


