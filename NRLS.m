clear; close all; clc;

% Parameters for the NLS algorithm
lambda = 0.97;  % Forgetting factor
P = eye(6);     % Initial covariance matrix (6x6 for two sources [x1, y1, z1, x2, y2, z2])
theta = zeros(6,1);  % Initial estimate of both sources' positions [x1, y1, z1, x2, y2, z2]

% Define the range and step size for equally spaced points
range_min = -100;
range_max = 100;
step_size = 9;  % Distance between points in each dimension

% Generate a grid of equally spaced points
[x_points, y_points, z_points] = meshgrid(range_min:step_size:range_max, ...
                                          range_min:step_size:range_max, ...
                                          range_min:step_size:range_max);

% Reshape the points into a list of coordinates (N x 3 matrix)
points = [x_points(:), y_points(:), z_points(:)];
N = size(points, 1);  % Number of points

% True source positions
p_source1_true = [20; 20; 0];  % True position of source 1
p_source2_true = [-10; -20; 0];  % True position of source 2

% Create synthetic data for the total magnetic field modulus H_tot
H_tot_data = zeros(N, 1);
for i = 1:N
    H1 = magnetic_field(points(i, :), p_source1_true);  % Magnetic field from source 1
    H2 = magnetic_field(points(i, :), p_source2_true);  % Magnetic field from source 2
    H_tot_data(i) = total_magnetic_field(H1, H2);       % Total magnetic field magnitude
end

% Placeholder for estimated positions over time
theta_history = zeros(N, 6);

% Nonlinear Recursive Least Squares Algorithm for Two Sources
for k = 1:N
    % Extract current estimate of both sources' positions
    p_source1_est = theta(1:3);  % Estimated position of source 1
    p_source2_est = theta(4:6);  % Estimated position of source 2

    % Compute magnetic fields for the estimated positions
    H1_est = magnetic_field(points(k, :), p_source1_est); 
    H2_est = magnetic_field(points(k, :), p_source2_est); 
    
    % Nonlinear function total_magnetic_field for current estimate
    H_est = total_magnetic_field(H1_est, H2_est);

    % Measurement innovation (error between measured and estimated output)
    y_tilde = H_tot_data(k) - H_est;

    % Unified approach for partial derivatives (same treatment for both sources)
    delta = 1e-6;  % Step size for finite difference
    phi_k = zeros(6, 1);  % Unified regression vector

    % Compute partial derivatives for both sources with respect to the total field
    for j = 1:3
        % Identity matrix for perturbation
        I = eye(3);

        % Perturb the position of source 1 and compute the total field
        perturbed_p1 = p_source1_est + delta * I(:,j);
        perturbed_H_tot_1 = total_magnetic_field(magnetic_field(points(k, :), perturbed_p1), magnetic_field(points(k, :), p_source2_est));
        phi_k(j) = (perturbed_H_tot_1 - H_est) / delta;

        % Perturb the position of source 2 and compute the total field
        perturbed_p2 = p_source2_est + delta * I(:,j);
        perturbed_H_tot_2 = total_magnetic_field(magnetic_field(points(k, :), p_source1_est), magnetic_field(points(k, :), perturbed_p2));
        phi_k(j+3) = (perturbed_H_tot_2 - H_est) / delta;
    end

    % Compute the gain vector
    K = P * phi_k / (lambda + phi_k' * P * phi_k);

    % Update parameter estimate (RLS update step)
    theta = theta + K * y_tilde;

    % Update the covariance matrix
    P = (P - K * phi_k' * P) / lambda;

    % Store the estimated parameters at each step
    theta_history(k, :) = theta';
end

% Plot true and estimated source positions
plot_estimates(theta_history, p_source1_true, p_source2_true, N);

% Compute and print errors
compute_and_print_errors(theta_history, p_source1_true, p_source2_true, N);

% -- Function Definitions --

% Function to compute magnetic field components from a source at p_source
function H = magnetic_field(point, p_source)
    % Rotation matrices assuming fixed pitch, roll, and yaw
    pitch = deg2rad(100); roll = deg2rad(25); yaw = deg2rad(90);

    Rx = [1, 0, 0; 0, cos(yaw), -sin(yaw); 0, sin(yaw), cos(yaw)];
    Ry = [cos(pitch), 0, sin(pitch); 0, 1, 0; -sin(pitch), 0, cos(pitch)];
    Rz = [cos(roll), -sin(roll), 0; sin(roll), cos(roll), 0; 0, 0, 1];

    R = Rz * Ry * Rx;
    
    % Relative position
    r = R.' * (point' - p_source);
    x = r(1); y = r(2); z = r(3);
    modulus = sqrt(x^2 + y^2 + z^2);
    modulus(modulus == 0) = eps; % Avoid division by zero
    
    % Compute the magnetic field components
    Hx = 300*x*z;
    Hy = 300*y*z;
    Hz = 200*z^2 - x^2 - y^2;
    
    % Return the magnetic field vector
    H = [Hx; Hy; Hz] / (modulus^5);
end

% Function to compute the total magnetic field magnitude
function H_tot = total_magnetic_field(H1, H2)
    % Magnitudes of the two fields
    norm_H1 = norm(H1);
    norm_H2 = norm(H2);
    
    % Angle between the two fields
    cos_alpha = dot(H1, H2) / (norm_H1 * norm_H2);
    
    % Total field magnitude using the given formula
    H_tot = sqrt(norm_H1^2 + norm_H2^2 + 2 * norm_H1 * norm_H2 * cos_alpha);
end

% Function to plot estimates vs true values
function plot_estimates(theta_history, p_source1_true, p_source2_true, N)
    figure;
    for i = 1:3
        subplot(3,2,i*2-1);
        plot(1:N, theta_history(:,i), 'r', 'LineWidth', 1.5); hold on;
        plot(1:N, repmat(p_source1_true(i), N, 1), 'k--', 'LineWidth', 2);
        xlabel('Time step');
        ylabel(['Estimate of source 1 ', char(119+i)]);
        legend('Estimated', 'True');
        title(['Source 1 ', char(119+i)]);
        
        subplot(3,2,i*2);
        plot(1:N, theta_history(:,i+3), 'b', 'LineWidth', 1.5); hold on;
        plot(1:N, repmat(p_source2_true(i), N, 1), 'k--', 'LineWidth', 2);
        xlabel('Time step');
        ylabel(['Estimate of source 2 ', char(119+i)]);
        legend('Estimated', 'True');
        title(['Source 2 ', char(119+i)]);
    end
end

% Function to compute and print absolute and relative errors
function compute_and_print_errors(theta_history, p_source1_true, p_source2_true, N)
    % Compute the final absolute errors for both sources
    x1_error = mean(abs(theta_history(:,1) - p_source1_true(1)));
    y1_error = mean(abs(theta_history(:,2) - p_source1_true(2)));
    z1_error = mean(abs(theta_history(:,3) - p_source1_true(3)));
    
    x2_error = mean(abs(theta_history(:,4) - p_source2_true(1)));
    y2_error = mean(abs(theta_history(:,5) - p_source2_true(2)));
    z2_error = mean(abs(theta_history(:,6) - p_source2_true(3)));
    
    % Print the final errors
    fprintf('Final mean absolute errors for Source 1: %.5f, %.5f, %.5f\n', x1_error, y1_error, z1_error);
    fprintf('Final mean absolute errors for Source 2: %.5f, %.5f, %.5f\n', x2_error, y2_error, z2_error);
end
