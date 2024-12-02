% PID Controller of Simplified Quadrotor Model
clear; close all; clc;
% Model Parmeters
global Ix Iy Iz g m J 
m = 4;                                   % Mass of the drone (kg)
g = 9.81;                                % Gravity (m/s^2)
Ix = 0.5; Iy = 0.5; Iz = 0.9; 
J = diag([Ix, Iy, Iz]);

% Time parameters
dt = 0.04;                  % Time step (s)
T_sim = 25;                 % Simulation time (s)
N_steps = T_sim / dt;       % Total number of steps
time = 0:dt:T_sim;          % Simulation time (s)

% Parameters for circular trajectory
radius = 5;        % Radius of the circular motion
cx = -radius;      % X-coordinate of the center
cy = 0;            % Y-coordinate of the center
cz = 5;            % Z-coordinate of the center

% Desired position function
desired_position = @(t) [cx + radius * cos(0.5 * t); ...
                         cy + radius * sin(0.5* t); ...
                         cz];

% Desired velocity function
desired_velocity = @(t) [-radius * 0.5 * sin(0.5 * t); ...
                          radius * 0.5 * cos(0.5 * t); ...
                          0];  

desired_acceleration = @(t) [-radius * 0.5^2 * cos(0.5 * t); ...
                             -radius * 0.5^2 * sin(0.5 * t); ...
                             0];

% Initial states
x = zeros(12, 1); % Initialize state vector
x(1:3) = desired_position(0); % Initial position
x(4:6) = desired_velocity(0); % Initial velocity
x(7:9) = [0 0 0]'; % Initial roll, pitch, yaw
x(10:12) = [0 0 0]'; % Initial angular velocity

% Controller gains
kxp = 0.3; kxd = 0.54;
kyp = 0.4; kyd = 0.6;
kzp = 1; kzd = 1;
kp_phi = 2;  kd_phi = 2.5;
kp_theta = 2;  kd_theta = 2.5;
kp_psi = 2;  kd_psi = 4;
gains = [kxp  kxd;
         kyp  kyd;
         kzp  kzd;
         kp_phi  kd_phi;
         kp_theta  kd_theta;
         kp_psi  kd_psi];

% Store results for plotting
state_history = zeros(12, N_steps);
time_history = zeros(1, N_steps);

% Initialize figure
figure_handle = init_plot(time, desired_position);

% Initialize handles
body_frame_handles = [];

% Simulation loop
for k = 1:length(time)
    t = time(k);

    % Check if figure is closed
    if ~isvalid(figure_handle)
        disp('Simulation stopped by user.');
        clf; cla; close;
        break;
    end

    % --- CONTROL STEP ---
    [input_torques, input_force] = control_laws(t, x, desired_position, desired_velocity, desired_acceleration, gains);
    
    % --- STATE UPDATE ---
    x_dot = simplified_model(dt, x, input_force, input_torques);
    x = x + x_dot * dt;

    % Store state and time
    state_history(:, k) = x;
    time_history(k) = t;
                                                   
    % --- Plot Drone and Frames ---
    % If body frame handles exist, delete them to update the plot
    if ~isempty(body_frame_handles)
        delete(body_frame_handles);
    end
    R = rotation_matrix(x(7), x(8), x(9));
    body_frame_handles = plot_drone(x(1:3), R, t);
end

% Extract results
plot_results(time, desired_position, state_history);

function x_dot = simplified_model(dt, state, F, input_torques)
    global Ix Iy Iz g m  
    v = state(4:6); phi = state(7); theta = state(8); psi = state(9); omega = state(10:12);
    
    % --- TRANSLATIONAL DYNAMICS ---
    vx_dot =  F / m * (cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi));
    vy_dot =  F / m * (cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi));
    vz_dot =  F / m * (cos(phi)*cos(theta)) - g;

    % --- ROTATIONAL DYNAMICS ---
    % Assuming small angles rpy_dot = omegaand R = R_dot
    p_dot = input_torques(1)/Ix;
    q_dot = input_torques(2)/Iy;
    r_dot = input_torques(3)/Iz;
    rpy_dot = omega;
    
    x_dot = [v; vx_dot; vy_dot; vz_dot; rpy_dot; p_dot; q_dot; r_dot];

end

function [input_torques, input_force] = control_laws(t, state, ...
    desired_position, desired_velocity, desired_acceleration, gains)
    global g m Ix Iy Iz 
    kxp = gains(1,1);  kxd = gains(1,2);
    kyp = gains(2,1);  kyd = gains(2,2);
    kzp = gains(3,1);  kzd = gains(3,2);
    kp_phi = gains(4,1);  kd_phi = gains(4,2);
    kp_theta = gains(5,1);  kd_theta = gains(5,2);
    kp_psi = gains(6,1);  kd_psi = gains(6,2);
    % States
    position = state(1:3);
    velocity = state(4:6);
    phi = state(7); theta = state(8); psi = state(9);
    phi_dot = state(10); theta_dot = state(11); psi_dot = state(12);

    % --- Desired Trajectory ---
    desired_pos = desired_position(t); % Desired position
    desired_vel = desired_velocity(t); % Desired velocity
    desired_acc = desired_acceleration(t); % Desired acceleration

    % --- Position Control (Outer Loop) ---
    % Compute position and velocity errors
    pos_error = desired_pos - position;  % Position error
    vel_error = desired_vel - velocity;  % Velocity error
    ex = pos_error(1); ey = pos_error(2); ez = pos_error(3);
    ex_dot = vel_error(1); ey_dot = vel_error(2); ez_dot = vel_error(3);

    % HEIGHT CONTROL
    input_force = m/(max(1e-6, (cos(phi) * cos(theta)))) * (g + kzp * ez + kzd * ez_dot);  % Altitude control (z)
    if input_force < 0 || isnan(input_force)
        error('Invalid input_force value: %f', input_force);
    end

    % POSITION CONTROL via PD
    % Compute desired thrust direction
    Fx = (m / max(1e-6, input_force)) * (kxp * ex + kxd * ex_dot + desired_acc(1));
    Fy = (m / max(1e-6, input_force)) * (kyp * ey + kyd * ey_dot + desired_acc(2));    

    % Compute desired roll (phi) and pitch (theta) from thrust direction
    phi_d = asin(max(-1, min(1, Fx * sin(psi) - Fy * cos(psi))));
    theta_d = asin(max(-1, min(1, (Fx * cos(psi) + Fy * sin(psi)) / max(1e-6, cos(phi_d)))));    
    psi_d = atan2(desired_vel(2), desired_vel(1));  % Desired yaw based on velocity (AGGIUNTO , NON C'E NELLA VENDIT)

    % Compute torques for attitude control
    tau_phi = Ix * (kp_phi * (phi_d - phi) + kd_phi * (-phi_dot));
    tau_theta = Iy * (kp_theta * (theta_d - theta) + kd_theta * (-theta_dot));
    tau_psi = Iz * (kp_psi * (psi_d - psi) + kd_psi * (-psi_dot));

    % Combine torques into a vector
    input_torques = [tau_phi; tau_theta; tau_psi];
end


function figure_handle = init_plot(time, desired_position)
    figure_handle = figure('Name', 'Drone Trajectory Simulation with Frames');
    hold on;
    grid on;
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    view(45, 30);
    axis([-15 15 -15 15 0 10]); % Set axis limits

    % Plot inertial frame (always visible)
    quiver3(0, 0, 0, 2, 0, 0, 'r', 'LineWidth', 2); % X-axis (red)
    quiver3(0, 0, 0, 0, 2, 0, 'g', 'LineWidth', 2); % Y-axis (green)
    quiver3(0, 0, 0, 0, 0, 2, 'b', 'LineWidth', 2); % Z-axis (blue)
    text(2, 0, 0, 'X_i', 'Color', 'r');
    text(0, 2, 0, 'Y_i', 'Color', 'g');
    text(0, 0, 2, 'Z_i', 'Color', 'b');

    % Plot desired trajectory
    t_full = time; % Time vector for the full trajectory
    desired_trajectory = arrayfun(@(t) desired_position(t), t_full, 'UniformOutput', false); % Compute trajectory points
    desired_trajectory = cell2mat(desired_trajectory); % Convert cell array to matrix
    plot3(desired_trajectory(1, :), desired_trajectory(2, :), desired_trajectory(3, :), 'k--', 'LineWidth', 1.5); % Plot trajectory
end

function body_frame_handles = plot_drone(body_origin, R, t)
    % Define the isosceles triangle in the body frame (relative to its center)
    triangle_body = [
        1.6, 0, 0;   % Tip of the triangle (pointing forward)
        -0.8, -0.4, 0; % Left base corner
        -0.8, 0.4, 0;  % Right base corner
        1.6, 0, 0;   % Close the triangle
    ];

    % Isosceles triangle marker for the drone
    triangle_inertial = (R * triangle_body')' + body_origin';

    % Plot UAV body frame
    scale = 1.5; % Scale for body frame axes
    body_x = body_origin + scale *  R(:, 1); % X-axis of body frame
    body_y = body_origin + scale *  R(:, 2); % Y-axis of body frame
    body_z = body_origin + R(:, 3); % Z-axis of body frame

    % Body frame axes
    body_frame_handles = [
        quiver3(body_origin(1), body_origin(2), body_origin(3), body_x(1)-body_origin(1), ...
                body_x(2)-body_origin(2), body_x(3)-body_origin(3), 'r', 'LineWidth', 1.5); % X-axis
        quiver3(body_origin(1), body_origin(2), body_origin(3), body_y(1)-body_origin(1), ...
                body_y(2)-body_origin(2), body_y(3)-body_origin(3), 'g', 'LineWidth', 1.5); % Y-axis
        quiver3(body_origin(1), body_origin(2), body_origin(3), body_z(1)-body_origin(1), ...
                body_z(2)-body_origin(2), body_z(3)-body_origin(3), 'b', 'LineWidth', 1.5); % Z-axis
        fill3(triangle_inertial(:, 1), triangle_inertial(:, 2), triangle_inertial(:, 3), 'b') % Triangle
    ];
    % Update plot title and view
    title(sprintf('Drone Trajectory Simulation with Frames - Time: %.2f s', t));
    pause(0.01);
end


function R = rotation_matrix(phi, theta, psi)
    % Compute the rotation matrix R from Euler angles (phi, theta, psi)
    % Standard case: Body frame's z-axis points downwards.

    % Rotation about z-axis (yaw)
    Rz = [cos(psi), -sin(psi), 0;
          sin(psi),  cos(psi), 0;
                0,        0, 1];

    % Rotation about y-axis (pitch)
    Ry = [cos(theta),  0, sin(theta);
                  0,  1,         0;
         -sin(theta), 0, cos(theta)];

    % Rotation about x-axis (roll)
    Rx = [1,        0,         0;
          0, cos(phi), -sin(phi);
          0, sin(phi),  cos(phi)];

    % Final rotation matrix
    R = Rz * Ry * Rx;
end


function plot_results(time, desired_position, state_history)
    % Extract data from state history
    x_hist = state_history(1, :);
    y_hist = state_history(2, :);
    z_hist = state_history(3, :);
    
    % Generate desired trajectory over time
    desired_traj = arrayfun(@(t) desired_position(t), time, 'UniformOutput', false);
    desired_traj = cell2mat(desired_traj);
    xd = desired_traj(1, :);
    yd = desired_traj(2, :);
    zd = desired_traj(3, :);

    % 3D Trajectory Plot
    figure;
    plot3(x_hist, y_hist, z_hist, 'b', 'LineWidth', 1.5); hold on;
    plot3(xd, yd, zd, 'r--', 'LineWidth', 1.5);
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    legend('Actual Trajectory', 'Desired Trajectory');
    title('3D Trajectory Comparison');
    grid on;

    % X, Y, Z Plots
    plot_single_dimension(time, x_hist, xd, 'X', 'X Position');
    plot_single_dimension(time, y_hist, yd, 'Y', 'Y Position');
    plot_single_dimension(time, z_hist, zd, 'Z', 'Z Position');
end

function plot_single_dimension(time, actual, desired, label, title_str)
    % Single dimension plot for X, Y, or Z
    figure;
    plot(time, actual, 'b', 'LineWidth', 1.5); hold on;
    plot(time, desired, 'r--', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel([label, ' (m)']);
    legend(['Actual ', label], ['Desired ', label]);
    title(title_str);
    grid on;
end


