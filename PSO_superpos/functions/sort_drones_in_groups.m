function [group_indices, n_groups] = sort_drones_in_groups(n_drones, n_sources) 
    % If n_drones is equal to n_groups, each group gets one drone. 
    % If n_drones > n_groups, the remaining drones are randomly assigned to the existing groups. 
    % If n_drones < n_groups, the drones are assigned in order.
    % Inputs:
    %   - n_drones:  Number of drones available for assignment.
    %   - n_sources:  Number of sources to which drones need to be assigned.
    % Output:
    %   - group_indices: A vector of size n_drones indicating the group each drone is assigned to.
    %   - n_groups : The obtained number of groups
    
        if n_drones >= n_sources
            % If there are equal or more drones than groups
            group_indices = (1:n_sources)';  % Assign at least one drone to each group
            if n_drones > n_sources
                % Randomly assign remaining drones to groups
                additional_assignments = randi(n_sources, n_drones - n_sources, 1);
                group_indices = [group_indices; additional_assignments];  % Concatenate assignments
                % Step 4: Sort to ensure grouping order
                group_indices = sort(group_indices);
            end
        else
            % If there are fewer drones than groups, assign drones only to a subset of groups
            group_indices = (1:n_drones)'; 
        end
        
        n_groups = max(group_indices);
    end
    