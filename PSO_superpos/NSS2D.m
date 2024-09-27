clear; close all; clc;

n_points = 100;
x = linspace(-100, 100, n_points);
y = linspace(-100, 100, n_points);
[x, y] = meshgrid(x, y); % Create 2D grid
mesh_x = x;
mesh_y = y;

% Define points as grid coordinates
points = [x(:)'; y(:)']; % Reshape the grid into a 2xN matrix of points

% Define source positions
p_sources = [10, 80 ,0; ... 
             30, 70 , 0;];

N_points = size(points, 2); % Number of points

% Preallocate for the grid of theta values
values = zeros(n_points, n_points);

% Loop through points and compute values
for i = 1:n_points
    for j = 1:n_points
        p_source1 = p_sources(1,:)';
        p_source2 = p_sources(2,:)';
        
        % Compute local points
        point_local1 = ([mesh_x(i,j); mesh_y(i,j); 0] - p_source1);
        point_local2 = ([mesh_x(i,j); mesh_y(i,j); 0] - p_source2); 
        
        % Compute Normalized Source Strength (NSS)
        NSS1 = NormalizedSourceStrength(point_local1);
        NSS2 = NormalizedSourceStrength(point_local2);
        %NSS1 = 0;
        
        % Sum of NSS values
        values(i,j) = NSS1 + NSS2;
    end
end


% Plotting the values on a 2D grid
figure;
contourf(x, y, values); % Create filled contour plot
xlabel('x'); ylabel('y'); zlabel('theta');
title('Theta Values on a 2D Grid');
colorbar; % Add a color bar to show color mapping

figure;
scatter(x(:), y(:), 70, values(:), 'filled');  % Points plot with color mapping
colorbar; % Add color bar to show value-to-color mapping
xlabel('x'); ylabel('y');
title('Points Colored According to Values');

% THIS FINDS ONLY 1 MAXIMUM
x_range = -100:0.5:100;  % X range of search
y_range = -100:0.5:100;  % Y range of search
[max_point, max_value] = gridSearchNSS(x_range, y_range, p_sources(1,1:2)');


function [max_point, max_value] = gridSearchNSS(x_range, y_range, source)
    % Grid search for maximum of NSS function
    max_value = -Inf;
    max_point = [0, 0];
    
    for x = x_range
        for y = y_range
            point = [x; y];
            point_local = point - source;
            value = NormalizedSourceStrength(point_local);
            
            if value > max_value
                max_value = value;
                max_point = point;
            end
        end
    end
    
    % Display the results
    fprintf('The maximum value of NSS occurs at point (x, y) = (%.4f, %.4f)\n', max_point(1), max_point(2));
    fprintf('The maximum value of NSS is: %.4f\n', max_value);
end


function NSS = NormalizedSourceStrength(point)
    % Extract x and y coordinates
    x = point(1);
    y = point(2);
    z = 0;
    C = 1;
    m = 1;
    
    % Calculate distance r
    r = sqrt(x^2 + y^2 + z^2);
    increase_factor = 100;

    % Prevent r from becoming too small (avoid division by near-zero)
    r_threshold = 1e-11;
    if r < r_threshold
        r = r_threshold;
    end
    
    % Calculate NSS
    NSS = increase_factor * C * m / r^4;
end








