function particles = init_drones(exclusion_zone_radius, n_drones, bounds, group_indices, max_velocity, velocity_randomness, inertia)
    % This function sets up n_drones in a circular formation with assigned goals 
    % based on their angles in space. Each drone is initialized with a position, 
    % velocity, and group index. The drones are assigned a goal based on their 
    % angle, allowing them to move in a specific direction.
    % Inputs:
    %   - n_drones: Number of drones to initialize.
    %   - bounds: Search space boundaries [min, max].
    %   - group_indices: Group assignment for each drone.
    %   - max_velocity: Maximum allowable velocity for each drone.
    %   - velocity_randomness: Factor controlling randomness in velocity.
    % Output:
    %   - particles: Cell array of initialized Particle objects (one per drone).
    
        % Initialize the particles array as a cell array
        particles = cell(n_drones, 1); 
    
        % Compute the angles for each drone
        angles = compute_angles(n_drones);  % Call the helper function
        
        max_distance = bounds(2)/2;  % Half of the boundary distance
    
        % Loop through each drone to initialize position, velocity, and goal
        for i = 1:n_drones
            % Get the angle for the current drone
            omega_i = angles(i);  % Extract the angle for this drone
            
            % Calculate the slope for the drone's movement path based on the angle
            m = tan(deg2rad(omega_i));  % Convert angle to radians and calculate slope
            
            % Assign a goal based on the angle, ensuring drones are spread around the space
            if (omega_i > 315 && omega_i <= 360) || omega_i <= 45
                % First and fourth quadrant (x positive, y positive/negative)
                goal = [max_distance, m * max_distance];
            elseif omega_i > 45 && omega_i <= 135
                % Second quadrant (x positive, y positive)
                goal = [max_distance / m, max_distance];
            elseif omega_i > 135 && omega_i <= 225
                % Third quadrant (x negative, y negative)
                goal = [-max_distance, m * -max_distance];
            elseif omega_i > 225 && omega_i <= 315
                % Fourth quadrant (x negative, y negative)
                goal = [-max_distance / m, -max_distance];
            end
            
            % Initialize particle with the computed goal and other parameters
            particles{i} = Particle(exclusion_zone_radius, goal, velocity_randomness, max_velocity, bounds, group_indices(i), i, n_drones, inertia);  
        end
    end
    
