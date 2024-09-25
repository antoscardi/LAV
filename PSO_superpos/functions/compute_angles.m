function angles = compute_angles(n_drones)
% This function calculates evenly spaced angles for each drone around a 360-degree circle.
% The angles are used to assign initial goals to each drone in the search space.
% Input:
%   - n_drones: Number of drones to compute angles for.
% Output:
%   - angles: A 1 x n_drones array containing the angle for each drone in degrees.

    % Compute the step size for each angle based on the number of drones
    omega = 360 / n_drones;          % Divide 360 degrees by the number of drones
    
    % Compute the angles for each drone
    angles = (0:n_drones-1) * omega; % Generate an array of angles evenly spaced
end