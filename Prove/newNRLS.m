clear; close all; clc;

n_points = 50;
x = linspace(-100, 100, n_points);
y = linspace(-100, 100, n_points);
[x, y] = meshgrid(x, y); % Create 2D grid
mesh_x = x;
mesh_y = y;

% Define points as grid coordinates
points = [x(:)'; y(:)']; % Reshape the grid into a 2xN matrix of points

% Rotation matrices
R1 = rotation_matrix(0, 0, 0); % No rotation
R2 = rotation_matrix(0, 0, 0); % No rotation

% Define source positions
p_sources = [0, 0 ,0; ... 
            20, 60 , 1;];

N_points = size(points, 2); % Number of points

% Preallocate for the grid of theta values
values = zeros(n_points, n_points);

% Loop through points and compute values
for i = 1:n_points
    for j = 1:n_points
        p_source1 = p_sources(1,:)';
        p_source2 = p_sources(2,:)';
        
        % Compute local points
        point_local1 = R1.' * ([mesh_x(i,j); mesh_y(i,j); 0] - p_source1);
        point_local2 = R2.' * ([mesh_x(i,j); mesh_y(i,j); 0] - p_source2); 
        
        % Compute gradient matrices for both sources
        G1 = gradient_matrix(point_local1);
        G2 = gradient_matrix(point_local2);
        
        % Compute Normalized Source Strength (NSS)
        NSS1 = NormalizedSourceStrength(G1);
        NSS2 = NormalizedSourceStrength(G2);
        
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
scatter(x(:), y(:), 100, values(:), 'filled');  % Points plot with color mapping
colorbar; % Add color bar to show value-to-color mapping
xlabel('x'); ylabel('y');
title('Points Colored According to Values');

function G = gradient_matrix(point)
    % Compute the gradient matrix (symmetric matrix for a source point)
    x = point(1);
    y = point(2);
    z = 0; % Assuming z is zero in the plane
    
    denominator = (x^2 + y^2 + z^2)^(7/2);
    
    % Components of the symmetric gradient matrix
    Gxx = x * (-2 * x^2 + 3 * y^2 + 3 * z^2);
    Gxy = y * (-4 * x^2 + y^2 + z^2);
    Gxz = z * (-4 * x^2 + y^2 + z^2);
    Gyx = Gxy;
    Gyy = x * (x^2 - 4 * y^2 + z^2);
    Gyz = -5 * x * y * z;
    Gzx = Gxz;
    Gzy = Gyz;
    Gzz = x * (x^2 + y^2 - 4 * z^2);
    
    % Assemble the symmetric matrix
    S = [Gxx, Gxy, Gxz;
         Gyx, Gyy, Gyz;
         Gzx, Gzy, Gzz];
     
    G = 3 / denominator * S; 
end

function NSS = NormalizedSourceStrength(G)
    % Compute eigenvalues
    lambdas = eig(G);
    disp(lambdas)
            
    % Sort eigenvalues in ascending order
    lambdas = sort(lambdas);

    % Extract eigenvalues
    lambda1 = lambdas(1);
    lambda2 = lambdas(2);
    lambda3 = lambdas(3);

    % Compute NSS using the given formula
    NSS = sqrt(-lambda2^2 - lambda1 * lambda3); 
end

function R = rotation_matrix(yaw, pitch, roll)
    % Compute the composite rotation matrix R = Rz * Ry * Rx
    R = [cos(roll)*cos(pitch), cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw), cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw);
         sin(roll)*cos(pitch), sin(roll)*sin(pitch)*sin(yaw)+cos(roll)*cos(yaw), sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw);
         -sin(pitch),          cos(pitch)*sin(yaw),                              cos(pitch)*cos(yaw)];
end
