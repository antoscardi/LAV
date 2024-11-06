clear; close all; clc;

% Define the number of sources and their positions (example with 3 sources)
n_sources = 3;
p_sources = [2, 2, 0]';% ...
%             -2, -2, 0; ...
%             4, 3, 0]'; % 3x3 matrix where each column is a source position

% Create rotations (identity for simplicity in this example)
R_array = cell(n_sources, 1);
R_array{1} = rotationMatrix(70, 0, 0);
R_array{2} = rotationMatrix(0, 90, 0);  
R_array{3} = rotationMatrix(0, 0, 115);

R_drones{1} = rotationMatrix(170, 0, 0);
R_drones{2} = rotationMatrix(0, 0, 30);  
R_drones{3} = rotationMatrix(0, 50, 0);

% Compute NSS at specified point, near a source
point = [0.1; 0.1; 0];
f = @(point) magnetic_field(point);
J_numerical = computeJacobianNumerically(f, point, R_array, R_drones,  p_sources);  % Numerical
checkSymmetry(J_numerical, 1, 0.1, 0.1, 0, 'numerical')
disp(J_numerical)
J_analytical = computeJacobianAnalytically(point, p_sources, R_array, R_drones);  % Analytical
disp(J_analytical)
checkSymmetry(J_analytical, 1, 0.1, 0.1, 0, 'analitycal')
nss_value_numerical = compute_nss(J_numerical);
nss_value_analytical = compute_nss(J_analytical);
disp(['NSS at the point using numerical Jacobian: ', num2str(nss_value_numerical)]);
disp(['NSS at the point using analytical Jacobian: ', num2str(nss_value_analytical)]);

% Define a grid of points in 3D space (e.g., x, y in [-5, 5] range, z = 1)
[x, y] = meshgrid(linspace(-5, 5, 40), linspace(-5, 5, 40)); % Increased resolution
z = 3; % Fixed drones to fly 5 meters above the ground

% Store NSS values for each point on the grid
nss_values = zeros(size(x));
jacobians = cell(size(x)); % Store the Jacobian matrices for each point (numerical)
jacobians_analytical = cell(size(x)); % Store the Jacobian matrices for each point (analytical)

% Compute the NSS and Jacobians at each point
for i = 1:numel(x)
    point = [x(i); y(i); z];
    
    % Compute the Jacobian numerically
    J_numerical = computeJacobianNumerically(f, point, R_array, R_drones,  p_sources);
    checkSymmetry(J_numerical, i, x, y, z, 'numerical')
    jacobians{i} = J_numerical;
    
    % Compute the Jacobian analytically
    J_analytical = computeJacobianAnalytically(point, p_sources, R_array, R_drones);
    checkSymmetry(J_analytical, i, x, y, z, 'analitycal')
    jacobians_analytical{i} = J_analytical;
    
    % Compute the NSS for the numerical Jacobian
    nss_values(i) = compute_nss(J_analytical);
end

% Plot the NSS values as a surface plot
figure('Name', 'NSS Distribution');
surf(x, y, nss_values, 'EdgeColor', 'none');
colorbar;
xlabel('x');
ylabel('y');
zlabel('NSS');
title('Normalized Source Strength (NSS) Distribution');
view(3); % View from above for a heatmap-like appearance

% Plot the numerical Jacobian components
figure('Name', 'Numerical Jacobian Components');
for row = 1:3
    for col = 1:3
        idx = (row - 1) * 3 + col;
        subplot(3, 3, idx);
        plotJacobianComponent(jacobians, x, y, row, col, getComponentName(row, col));
    end
end

% Plot the analytical Jacobian components
figure('Name', 'Analytical Jacobian Components');
for row = 1:3
    for col = 1:3
        idx = (row - 1) * 3 + col;
        subplot(3, 3, idx);
        plotJacobianComponent(jacobians_analytical, x, y, row, col, getComponentName(row, col));
    end
end

% Compute and plot the differences between the numerical and analytical Jacobians
differences = cell(size(x));
for i = 1:numel(x)
    differences{i} = jacobians{i} - jacobians_analytical{i};
end

% Display the total difference between numerical and analytical Jacobians
total_difference = 0;
for i = 1:numel(x)
    total_difference = total_difference + norm(differences{i}, 'fro');
end
disp('Total difference between numerical and analytical Jacobians:');
disp(total_difference/numel(x));

disp("Jacobian numerical example")
disp(jacobians{1})
disp("Jacobian analytical example")
disp(jacobians_analytical{1})


% Function to define a 3D rotation matrix from roll, pitch, and yaw angles (in radians)
function R = rotationMatrix(roll, pitch, yaw)
    roll = deg2rad(roll);
    pitch = deg2rad(pitch);
    yaw = deg2rad(yaw);
    % Rotation about the X-axis (roll)
    Rx = [1, 0, 0;
          0, cos(roll), -sin(roll);
          0, sin(roll), cos(roll)];
      
    % Rotation about the Y-axis (pitch)
    Ry = [cos(pitch), 0, sin(pitch);
          0, 1, 0;
          -sin(pitch), 0, cos(pitch)];
      
    % Rotation about the Z-axis (yaw)
    Rz = [cos(yaw), -sin(yaw), 0;
          sin(yaw), cos(yaw), 0;
          0, 0, 1];
      
    % Combined rotation matrix (R = Rz * Ry * Rx)
    R = Rz * Ry * Rx;
end


% Function to compute the total magnetic field from multiple sources at a given point
function vector_field = magnetic_field(point)
    % Calculate the magnetic field using the given formula
    x = point(1);
    y = point(2);
    z = point(3);
    
    r = sqrt(x^2 + y^2 + z^2);
    r(r == 0) = 1e-11;  % Avoid division by zero

    %fprintf('r with respect to source: %.1f, Source position: [%.1f, %.1f, %.1f]\n', r, p_sources(1, i), p_sources(2, i), p_sources(3, i));
    Hx = (3 * x * z) / r^5;
    Hy = (3 * y * z) / r^5;
    Hz = (3 * z^2 / r^2 - 1) / r^3;
    
    % Add the contribution to the total field
    vector_field = [Hx; Hy; Hz];
end

function J = computeJacobianNumerically(f, global_point, R_array, R_drones,  p_sources)
    delta = 1e-5; % Small perturbation for numerical differentiation
    J = zeros(3, 3); % Initialize the Jacobian matrix
    % Loop over each direction (x, y, z) for perturbation
    for k = 1:3
        perturbation = zeros(3, 1);
        perturbation(k) = delta; % Apply perturbation in the local frame
        % Initialize the total fields at perturbed points
        f_plus_total = zeros(3, 1);
        f_minus_total = zeros(3, 1);
        % Loop over all sources
        for i = 1:size(p_sources, 2)
            % Compute the point in local coordinates for the current source
            point_local = R_array{i}.' * (global_point - p_sources(:, i));
            % Apply the perturbation in the local frame
            point_plus_local = point_local + perturbation;
            point_minus_local = point_local - perturbation;
            % Compute the field at the perturbed points for the current source
            f_plus = f(point_plus_local);   % f(point + delta) for this source
            f_minus = f(point_minus_local); % f(point - delta) for this source
            % Sum the contributions from the current source
            f_plus_total = f_plus_total + f_plus;
            f_minus_total = f_minus_total + f_minus;
        end
        % Compute the derivative using the total fields across all sources
        J(:, k) = (f_plus_total - f_minus_total) / (2 * delta);
    end
    J = R_drones{i} * J * R_drones{i}.';
end

% Function to compute the Jacobian matrix analytically
function J = computeJacobianAnalytically(point, p_sources, R_array, R_drones)
    n_sources = size(p_sources, 2);
    J = zeros(3, 3); % Initialize the Jacobian matrix
    
    % Loop over each source to compute its contribution to the Jacobian
    for i = 1:n_sources
        % Compute the local point relative to the source
        local_point = R_array{i}' * (point - p_sources(:, i));
        
        x = local_point(1);
        y = local_point(2);
        z = local_point(3);

        denominator = (x^2 + y^2 + z^2)^(7/2);

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
        J_source = [dHx_dx, dHx_dy, dHx_dz;
                    dHy_dx, dHy_dy, dHy_dz;
                    dHz_dx, dHz_dy, dHz_dz];

        % Add the source's Jacobian to the total Jacobian
        J = J + R_drones{i}.' * J_source * R_drones{i};
    end
end

% Function to compute the NSS given a Jacobian matrix
function nss = compute_nss(J)
    % Compute eigenvalues of the Jacobian matrix
    eigenvalues = eig(J);

    % Sort eigenvalues in ascending order
    sorted_eigenvalues = sort(eigenvalues);

    % Extract the sorted eigenvalues
    eigen_min = sorted_eigenvalues(1);
    eigen_middle = sorted_eigenvalues(2);
    eigen_max = sorted_eigenvalues(3);

    % Compute the NSS using the formula
    nss = sqrt(-eigen_middle^2 - eigen_max * eigen_min);
    nss = real(nss);

    % Instead of NSS use Frobenius norm
    %nss = norm(J, 'fro');
end

% Function to plot a single component of the Jacobian
function plotJacobianComponent(jacobians, x, y, row, col, componentName)
    % Extract the specified component for Jacobians
    component = arrayfun(@(i) jacobians{i}(row, col), 1:numel(x));
    
    % Reshape for plotting
    component = reshape(component, size(x));
    
    % Plot the component as a 3D surface
    surf(x, y, component);
    title(['$', componentName, '$'], 'Interpreter', 'latex');
    xlabel('x'); ylabel('y'); zlabel('Value');
    colorbar; % Add a colorbar for better understanding of values
end

% Function to get the name of the Jacobian component in LaTeX format
function name = getComponentName(row, col)
    % Define names for each component based on row and column using LaTeX formatting
    components = {'\partial H_x / \partial x', '\partial H_x / \partial y', '\partial H_x / \partial z'; ...
                  '\partial H_y / \partial x', '\partial H_y / \partial y', '\partial H_y / \partial z'; ...
                  '\partial H_z / \partial x', '\partial H_z / \partial y', '\partial H_z / \partial z'};
    name = components{row, col};
end


% Function to check if a matrix is symmetric with a tolerance and print an error if not
function checkSymmetry(J, index, x, y, z, method)
    % Define a small tolerance for numerical comparisons
    tolerance = 1e-3;
    
    % Check if the matrix is symmetric within the specified tolerance
    if any(any(abs(J - J') > tolerance))
        fprintf('Error: %s Jacobian at index %.1d (x=%.1f, y=%.1f, z=%.1f) is not symmetric within tolerance.\n', method, index, x, y, z);
        % Optionally, you can display the difference matrix to understand the asymmetry:
        % disp('Difference matrix (J - J'') with tolerance check:');
        % disp(J - J');
    end
end
