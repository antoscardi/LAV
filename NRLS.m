clear; close all; clc 
% Define parameters for the EKF algorithm
lambda = 0.99;  % Forgetting factor
P = eye(3);     % Initial covariance matrix (3x3 for x, y, z)
theta = zeros(3,1);  % Initial estimate of source position [x; y; z]
Q = 0.01 * eye(3); % Process noise covariance
R = 0.1; % Measurement noise variance

% Define the range and step size for equally spaced points
range_min = -100;
range_max = 100;
step_size = 10;  % Distance between points in each dimension

% Generate a grid of equally spaced points
[x_points, y_points, z_points] = meshgrid(range_min:step_size:range_max, ...
                                          range_min:step_size:range_max, ...
                                          range_min:step_size:range_max);

% Reshape the points into a list of coordinates (N x 3 matrix)
points = [x_points(:), y_points(:), z_points(:)];
N = size(points, 1);  % Update N to reflect the number of generated points
disp(N)

% True source position and orientation (yaw, pitch, roll)
p_source_true = [10; 20; 30];  % True source position
yaw_true = pi/6;  % 30 degrees
pitch_true = pi/4;  % 45 degrees
roll_true = pi/3;  % 60 degrees

% Create synthetic data for the magnetic field modulus H_modulus
H_modulus_data = zeros(N, 1);
for i = 1:N
    H_modulus_data(i) = mod_H(points(i, :), p_source_true, yaw_true, pitch_true, roll_true) + 0.1 * randn;
end

% Placeholder for estimated source position over time
theta_history = zeros(N, 3);

% Extended Kalman Filter Algorithm
for k = 1:N
    % Extract current estimate of source position
    p_source_est = theta;

    % Nonlinear function mod_H for current estimate
    H_est = mod_H(points(k, :), p_source_est, yaw_true, pitch_true, roll_true);

    % Measurement innovation (error between measured and estimated output)
    y_tilde = H_modulus_data(k) - H_est;

    % Create the Jacobian matrix (partial derivatives w.r.t source position x, y, z)
    delta = 1e-4;  % Step size for finite difference
    dH_dx = (mod_H(points(k, :), p_source_est + [delta; 0; 0], yaw_true, pitch_true, roll_true) - H_est) / delta;
    dH_dy = (mod_H(points(k, :), p_source_est + [0; delta; 0], yaw_true, pitch_true, roll_true) - H_est) / delta;
    dH_dz = (mod_H(points(k, :), p_source_est + [0; 0; delta], yaw_true, pitch_true, roll_true) - H_est) / delta;
    
    % Create the Jacobian (measurement model linearization)
    H_jacobian = [dH_dx, dH_dy, dH_dz];

    % Kalman gain
    S = H_jacobian * P * H_jacobian' + R;  % Measurement covariance
    K = P * H_jacobian' / S;  % Kalman gain

    % Update state estimate (source position)
    theta = theta + K * y_tilde;

    % Update covariance estimate
    P = (eye(3) - K * H_jacobian) * P;

    % Store the estimated parameters at each step
    theta_history(k, :) = theta';
end

% Plot true and estimated source positions over time
figure;
subplot(3,1,1);
plot(1:N, theta_history(:,1), 'r', 'LineWidth', 1.5); hold on;
plot(1:N, repmat(p_source_true(1), N, 1), 'k--', 'LineWidth', 2);
xlabel('Time step');
ylabel('Estimate of x');
title('Source Position x Estimation');
legend('Estimated x', 'True x');

subplot(3,1,2);
plot(1:N, theta_history(:,2), 'b', 'LineWidth', 1.5); hold on;
plot(1:N, repmat(p_source_true(2), N, 1), 'k--', 'LineWidth', 2);
xlabel('Time step');
ylabel('Estimate of y');
title('Source Position y Estimation');
legend('Estimated y', 'True y');

subplot(3,1,3);
plot(1:N, theta_history(:,3), 'g', 'LineWidth', 1.5); hold on;
plot(1:N, repmat(p_source_true(3), N, 1), 'k--', 'LineWidth', 2);
xlabel('Time step');
ylabel('Estimate of z');
title('Source Position z Estimation');
legend('Estimated z', 'True z');

% Compute the final absolute errors
x_error = mean(abs(theta_history(:,1) - p_source_true(1)));
y_error = mean(abs(theta_history(:,2) - p_source_true(2)));
z_error = mean(abs(theta_history(:,3) - p_source_true(3)));

% Compute the relative errors
x_relative_error = mean(abs((theta_history(:,1) - p_source_true(1)) / p_source_true(1)));
y_relative_error = mean(abs((theta_history(:,2) - p_source_true(2)) / p_source_true(2)));
z_relative_error = mean(abs((theta_history(:,3) - p_source_true(3)) / p_source_true(3)));

% Print the final errors and relative errors
fprintf('Final mean absolute errors:\n');
fprintf('x error: %.5f\n', x_error);
fprintf('y error: %.5f\n', y_error);
fprintf('z error: %.5f\n', z_error);

fprintf('Final mean relative errors:\n');
fprintf('x relative error: %.5f\n', x_relative_error);
fprintf('y relative error: %.5f\n', y_relative_error);
fprintf('z relative error: %.5f\n', z_relative_error);


% Function definition for mod_H
function H_modulus = mod_H(point, p_source, yaw, pitch, roll)
    % Compute the rotation matrices
    Rx = [1, 0, 0;
          0, cos(yaw), -sin(yaw);
          0, sin(yaw), cos(yaw)];

    Ry = [cos(pitch), 0, sin(pitch);
          0, 1, 0;
          -sin(pitch), 0, cos(pitch)];

    Rz = [cos(roll), -sin(roll), 0;
          sin(roll), cos(roll), 0;
          0, 0, 1];
      
    R = Rz * Ry * Rx;
    
    % Compute p_local
    p_local = R.' * (point' - p_source);
    
    % Extract local x, y, z coordinates
    x = p_local(1);
    y = p_local(2);
    z = p_local(3);
    
    % Compute r, the norm of the position vector
    r = sqrt(x^2 + y^2 + z^2);
    
    % Ensure that r does not get too close to zero
    if r < 1e-6
        r = 1e-6;
    end
    
    % Combine components and simplify the final expression
    H_squared = (4*x^4 + y^4 + z^4 + 5*x^2*y^2 + 5*x^2*z^2 + 2*y^2*z^2)/ r^5;
    
    % Compute the modulus of the magnetic field
    H_modulus = 1/sqrt(H_squared);
end
