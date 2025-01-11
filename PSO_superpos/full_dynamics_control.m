% PID Controller of FULL Quadrotor Model
clear; close all; clc;
%% Define colors
global ocra orange green gray red blue 
ocra = [1, 0.8, 0];          % Ocra color (hex: #ffcc00)
orange = [0.922, 0.38, 0.24]; % Orange color (hex: #eb613d)
gray = [0.75, 0.75, 0.75];   % Gray color (hex: #C0C0C0)
red = [139,0,0]/255; % Red color (RGB: 178,34,34)
blue = [0,0,205]/255;    % Blue color (RGB: 30,144,255)
green = [0,128,0]/255; % Green color (RGB: 60,179,113)
% Sizes of text
global title_font_size label_font_size legend_font_size
title_font_size = 20;
label_font_size = 17;
legend_font_size = 18;
% Create a VideoWriter object with high-quality settings
dt = 0.04;                  % Time step (s)
do_video = true;
if do_video
    video_filename = 'figures/quadrotor_simulation.mp4';
    video = VideoWriter('figures/quadrotor_simulation', 'Uncompressed AVI');
    video.FrameRate = 1 / dt; % Match simulation time step
    open(video);
end
% Model Parmeters
global Ix Iy Iz g m J l c_t c rotor_speed_max rotor_speed_min radius
%% ---- SMALL QUADROTOR ref:https://hal.science/hal-02491491/document
% m = 1.285;                                  % Mass of the drone (kg)                         
% Ix = 0.0188; Iy = 0.0193; Iz = 0.0288;
% l = 0.17;                                  % Length arms (m), distance from center to propellers
% c_t = 3.68e-6;                             % Force/Thrust coefficient
% c_d = 5.11e-8;                             % Torque coefficient (DRAG/MOMENT)                            
%% ---- VERY LARGE QUADROTOR ref:Modelling and control of a large quadrotor robot
% m = 4.34;                                  % Mass of the drone (kg)                               
% Ix = 0.0820; Iy = 0.0845; Iz = 0.1377;
% l = 0.35;                                  % Length arms (m), distance from center to propellers
% c_t = 0.0047;                              % Force/Thrust coefficient
% c_d = 0.228e-3;                            % Torque coefficient (DRAG/MOMENT)
%% ---- ANOTHER ALSO SMALL DRAGONFLY ref:https://www.ijmerr.com/uploadfile/2020/0519/20200519103601235.pdf
% m = 0.53;                                  % Mass of the drone (kg)                         
% Ix = 5e-3; Iy = 5e-3; Iz = 8.9e-3;
% l = 0.225;                                 % Length arms (m), distance from center to propellers
% c_t = 6.317e-4;                            % Force/Thrust coefficient
% c_d = 1.61e-4;                             % Torque coefficient (DRAG/MOMENT)   
%% ---- ANOTHER ALSO SMALL DRAGONFLY IV ref:https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=4237837
m = 0.4;                                  % Mass of the drone (kg)                         
Ix =  3.8278e-3; Iy = 3.8278e-3; Iz = 7.1345e-3;
l = 0.205;                                 % Length arms (m), distance from center to propellers
c_t =  6.354e-4;                            % Force/Thrust coefficient
c_d = 0.048;                               % Torque coefficient (DRAG/MOMENT)
  
%%--------------------------------
g = 9.81;                                 % Gravity (m/s^2)
J = diag([Ix, Iy, Iz]);
c = c_d/c_t;  
rotor_speed_min = 0;                      % rad/s
rotor_speed_max = 95;                     % rad/s
radius = 0.1; % 10cm JUST FOR PLOTTING 


% Time parameters
T_sim = 25;                 % Simulation time 25 (s)
N_steps = T_sim / dt;       % Total number of steps
time = 0:dt:T_sim;          % Simulation time (s)

% % Parameters for circular trajectory
%  rad = 5;        % Radius of the circular motion
%  cx = -radius;      % X-coordinate of the center
%  cy = 0;            % Y-coordinate of the center
%  cz = 5;            % Z-coordinate of the center

% % Desired position function
%  desired_position = @(t) [cx + rad * cos(0.5 * t); ...
%                           cy + rad * sin(0.5* t); ...
%                           cz];
% % Desired velocity function
%  desired_velocity = @(t) [-rad * 0.5 * sin(0.5 * t); ...
%                            rad * 0.5 * cos(0.5 * t); ...
%                            0];  

% desired_acceleration = @(t) [-rad * 0.5^2 * cos(0.5 * t); ...
%                               -rad * 0.5^2 * sin(0.5 * t); ...
%                               0];

 % Parameters
 z_max = 5;      % Maximum altitude
 k = 0.5;        % Smoothness factor for exponential rise
 v = 1.5;          % Constant velocity in x-y plane
 angle = pi/4;   % Angle of projection in x-y plane (45 degrees)

 % Define desired position as a function of time
 desired_position = @(t) [
     v * t * cos(angle);                     % x(t): Linear motion in x
     v * t * sin(angle);                     % y(t): Linear motion in y
     z_max * (1 - exp(-k * t))               % z(t): Smooth rise
 ];

 % Define desired velocity as a function of time
 desired_velocity = @(t) [
     v * cos(angle);                         % x velocity
     v * sin(angle);                         % y velocity
     z_max * k * exp(-k * t)                 % z velocity
 ];

 % Define desired acceleration as a function of time
 desired_acceleration = @(t) [
     0;                                      % x acceleration
     0;                                      % y acceleration
     -z_max * k^2 * exp(-k * t)              % z acceleration
 ];

% Initial states
x = zeros(12, 1); % Initialize state vector
x(1:3) = desired_position(0); % Initial position
x(4:6) = [0 0 0]'; % Initial velocity
x(7:9) = [0 0 0]'; % Initial roll, pitch, yaw
x(10:12) = [0 0 0]'; % Initial angular velocity

% Controller gains
kxp = 0.1; kxd = 0.5;
kyp = 0.1; kyd = 0.54;
kzp = 1; kzd = 2.5;
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
rotor_velocities_history = zeros(4, N_steps);
rotor_forces_history = zeros(4, N_steps);

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
    x_dot = full_model(dt, x, input_force, input_torques);
    x = x + x_dot * dt;
    %disp(x)

    % Compute rotor velocities
    rotor_velocities = compute_rotor_velocities(input_force, input_torques);
    rotor_forces = compute_rotor_forces(rotor_velocities);

    % Store state and time
    state_history(:, k) = x;
    time_history(k) = t;
    rotor_velocities_history(:, k) = rotor_velocities;
    rotor_forces_history(:,k) = rotor_forces;
                                                   
    % --- Plot Drone and Frames ---
    % If body frame handles exist, delete them to update the plot
    if ~isempty(body_frame_handles)
        delete(body_frame_handles);
    end
    R = rotation_matrix(x(7), x(8), x(9));
    body_frame_handles = plot_drone(x(1:3), R, t);
    if t == 7 
        exportgraphics(gcf, fullfile('figures', 'pid_sim.pdf'), 'ContentType', 'vector');
    end
    if do_video
        frame = getframe(figure_handle); % Capture high-resolution frame
        frame_resized = imresize(frame.cdata, [995, 1910]);
        writeVideo(video, frame_resized);
    end
end

if do_video
    close(video);
    disp(['High-quality video saved as ', video_filename]);
end

% Extract results
plot_results(time, desired_position, state_history, rotor_velocities_history, rotor_forces_history);
% Save all other plots generated in the simulation
exportgraphics(figure(2), fullfile('figures', 'plot_2.pdf'),'ContentType', 'vector');
exportgraphics(figure(3), fullfile('figures', 'plot_3.pdf'),'ContentType', 'vector');
exportgraphics(figure(4), fullfile('figures', 'plot_4.pdf'),'ContentType', 'vector');
exportgraphics(figure(5), fullfile('figures', 'plot_5.pdf'),'ContentType', 'vector');
exportgraphics(figure(6), fullfile('figures', 'plot_6.pdf'),'ContentType', 'vector');
%saveas(figure(2), 'figures/plot_2.png');
%saveas(figure(3), 'figures/plot_3.png');
%saveas(figure(4), 'figures/plot_4.png');
%saveas(figure(5), 'figures/plot_5.png');
%saveas(figure(6), 'figures/plot_6.png');


function x_dot = full_model(dt, state, F, input_torques)
    global Ix Iy Iz g m  
    v = state(4:6); phi = state(7); theta = state(8); psi = state(9);
    p = state(10); q = state(11); r = state(12);
    
    % --- TRANSLATIONAL DYNAMICS ---
    vx_dot =  F / m * (cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi));
    vy_dot =  F / m * (cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi));
    vz_dot =  F / m * (cos(phi)*cos(theta)) - g;

    % --- ROTATIONAL DYNAMICS ---
    p_dot =  q*r*(Iy-Iz)/Ix + input_torques(1)/Ix;
    q_dot =  p*r*(Iz-Ix)/Iy + input_torques(2)/Iy;
    r_dot =  p*q*(Ix-Iy)/Iz + input_torques(3)/Iz;
    phi_dot = p + sin(phi) * tan(theta) *q + cos(phi) * tan(theta) *r;
    theta_dot = cos(phi) * q - sin(phi)* r;
    psi_dot = sin(phi) * sec(theta) * q + cos(phi) * sec(theta) * r;
    
    x_dot = [v; vx_dot; vy_dot; vz_dot; phi_dot; theta_dot; psi_dot; p_dot; q_dot; r_dot];

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
    phi_d = asin(Fx * sin(psi) - Fy * cos(psi));
    theta_d = asin(Fx * cos(psi) + Fy * sin(psi)) / max(1e-6, cos(phi_d));    
    psi_d = atan2(desired_vel(2), desired_vel(1));  % Desired yaw based on velocity (AGGIUNTO, NON C'E NELLA VENDIT)

    % Compute torques for attitude control
    tau_phi = Ix * (kp_phi * (phi_d - phi) + kd_phi * (-phi_dot));
    tau_theta = Iy * (kp_theta * (theta_d - theta) + kd_theta * (-theta_dot));
    tau_psi = Iz * (kp_psi * (psi_d - psi) + kd_psi * (-psi_dot));

    % Combine torques into a vector
    input_torques = [tau_phi; tau_theta; tau_psi];
end

function rotor_velocities = compute_rotor_velocities(F, torques)
    global c_t l c 
    % Solve for rotor speeds squared
    A = c_t * [
        1, 1, 1, 1;              % Total thrust
        0, -l, 0, l;             % Roll torque
        l, 0, -l, 0;             % Pitch torque
        -c, +c, -c, +c;          % Yaw torque
    ];
    b_vector = [F; torques];
    rotor_velocities = sqrt(A \ b_vector);

    % Ensure no negative rotor speeds
    if any(rotor_velocities < 0)
        disp(rotor_velocities)
        error("Negative speed")
    end

    if any(rotor_velocities > 260)
        disp(rotor_velocities)
        disp("Speed more than 700 rad/s")
        rotor_velocities(rotor_velocities > 260) = 260;
    end    
end

function rotor_forces = compute_rotor_forces(rotor_velocities)
    global c_t
    
    rotor_forces = rotor_velocities.^2/c_t;

    % Ensure no negative rotor speeds
    if any(rotor_forces < 0)
        disp(rotor_forces)
        error("Negative force")
    end
end

function figure_handle = init_plot(time, desired_position)
    global red blue green title_font_size label_font_size legend_font_size
    
    figure_handle = figure('Name', 'Drone Trajectory Simulation', 'NumberTitle', 'off');
    set(figure_handle, 'Units', 'pixels', 'Position', [100, 100, 1910, 995]); % Fixed size
    hold on;
    grid on;
    xlabel('$x$ [m]', 'Interpreter', 'latex', 'FontSize', label_font_size+6);
    ylabel('$y$ [m]', 'Interpreter', 'latex', 'FontSize', label_font_size+6);
    zlabel('$z$ [m]', 'Interpreter', 'latex', 'FontSize', label_font_size+6);
    view(15.6875,37.9459)
    %view(45, 30);
    %view(0, 90); % view from above
    %axis([0 40 0 40 0 8]); % Set axis limits

    % Plot inertial frame (always visible)
    quiver3(0, 0, 0, 4, 0, 0, 'Color', red, 'LineWidth', 2.5); % X-axis 
    quiver3(0, 0, 0, 0, 7, 0, 'Color', blue, 'LineWidth', 2.5); % Y-axis 
    quiver3(0, 0, 0, 0, 0, 1, 'Color', green, 'LineWidth', 2.5); % Z-axis 
    text(4, 0, 0, '$x_i$', 'Color', red, 'FontSize', label_font_size, 'Interpreter', 'latex');
    text(0, 7.1, 0, '$y_i$', 'Color', blue, 'FontSize', label_font_size, 'Interpreter', 'latex');
    text(0, 0, 1.1, '$z_i$', 'Color', green, 'FontSize', label_font_size, 'Interpreter', 'latex');

    % Plot desired trajectory
    t_full = time; % Time vector for the full trajectory
    desired_trajectory = arrayfun(@(t) desired_position(t), t_full, 'UniformOutput', false); % Compute trajectory points
    desired_trajectory = cell2mat(desired_trajectory); % Convert cell array to matrix
    plot3(desired_trajectory(1, :), desired_trajectory(2, :), desired_trajectory(3, :), 'k--', 'LineWidth', 2); % Plot trajectory
end

function body_frame_handles = plot_drone(body_origin, R, t)
    global l radius red blue green ocra gray orange title_font_size label_font_size
    INC = 10; %INCREASED SIZE FOR VISUALIZATION PURPOSES ONLY

    % Define the isosceles triangle in the body frame (relative to its center)
    arms_body = [
        INC*l, 0, 0;  % Arm 1 endpoint (x direction)
        -INC*l, 0, 0; % Arm 2 endpoint (-x direction)
        0, INC*l, 0;  % Arm 3 endpoint (y direction)
        0, -INC*l, 0; % Arm 4 endpoint (-y direction)
    ];
    % Compute endpoints of arms in the inertial frame
    arms_inertial = (R * arms_body')' + body_origin';

    % Plot UAV body frame
    body_x = body_origin +  5 * R(:, 1); % X-axis of body frame
    body_y = body_origin +  6 * R(:, 2); % Y-axis of body frame
    body_z = body_origin +  1 * R(:, 3); % Z-axis of body frame

    % Body frame axes
    body_frame_handles = [
        quiver3(body_origin(1), body_origin(2), body_origin(3), body_x(1)-body_origin(1), ...
                body_x(2)-body_origin(2), body_x(3)-body_origin(3), 'Color', red, 'LineWidth', 3); % X-axis
        quiver3(body_origin(1), body_origin(2), body_origin(3), body_y(1)-body_origin(1), ...
                body_y(2)-body_origin(2), body_y(3)-body_origin(3), 'Color', blue, 'LineWidth', 3); % Y-axis
        quiver3(body_origin(1), body_origin(2), body_origin(3), body_z(1)-body_origin(1), ...
                body_z(2)-body_origin(2), body_z(3)-body_origin(3), 'Color', green, 'LineWidth', 3); % Z-axis
        text(body_x(1), body_x(2), body_x(3), '$x_r$', 'Color', red, 'FontSize', label_font_size, 'Interpreter', 'latex');
        text(body_y(1), body_y(2), body_y(3), '$y_r$', 'Color', blue, 'FontSize', label_font_size, 'Interpreter', 'latex');
        text(body_z(1), body_z(2), body_z(3), '$z_r$', 'Color', green, 'FontSize', label_font_size, 'Interpreter', 'latex');
    ];

    for i = 1:size(arms_body, 1)
        % Determine color based on arm direction
        if i == 1 || i == 2 % Front and back arms (x direction)
            arm_color = orange;
        else % Left and right arms (y direction)
            arm_color = ocra;
        end
        % Draw lines for arms
        line_handle = line([body_origin(1), arms_inertial(i, 1)], ...
            [body_origin(2), arms_inertial(i, 2)], ...
            [body_origin(3), arms_inertial(i, 3)], ...
            'Color', [0, 0, 0, 0.8], 'LineWidth', 2);
        % Draw circles at arm endpoints (rotated with the body)
        theta = linspace(0, 2*pi, 100);
        circle_body = INC*radius * [cos(theta); sin(theta); zeros(size(theta))]; % Circle in body frame
        circle_inertial = (R * circle_body)'; % Rotate circle to inertial frame
        circle_x = circle_inertial(:, 1) + arms_inertial(i, 1);
        circle_y = circle_inertial(:, 2) + arms_inertial(i, 2);
        circle_z = circle_inertial(:, 3) + arms_inertial(i, 3);
        circle_handle = fill3(circle_x, circle_y, circle_z, arm_color);
        % Append to body frame handles
        body_frame_handles = [body_frame_handles; line_handle; circle_handle];
    end
    % Update plot title and view
    title(sprintf('Drone Trajectory Simulation - Time: %.2f s', t), ...
        'FontSize', title_font_size, 'Interpreter', 'latex');
    pause(0.02);
end

function R = rotation_matrix(phi, theta, psi)
    % Compute the rotation matrix R from Euler angles (phi, theta, psi)
    % Standard case: Body frame's z-axis points upwards.

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

function plot_results(time, desired_position, state_history, rotor_velocities_history, rotor_forces_history)
    global rotor_speed_max rotor_speed_min title_font_size label_font_size legend_font_size
    global red green blue orange
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
    set(gcf,'paperposition',[0 0 8 5],'papersize',[8 5])
    plot3(x_hist, y_hist, z_hist, 'Color', blue,'LineWidth', 1.5); hold on;
    plot3(xd, yd, zd, '--', 'Color', green,  'LineWidth', 1.5);
    xlabel('$x$ [m]', 'Interpreter', 'latex', 'FontSize', label_font_size);
    ylabel('$y$ [m]', 'Interpreter', 'latex', 'FontSize', label_font_size);
    zlabel('$z$ [m]', 'Interpreter', 'latex', 'FontSize', label_font_size);
    legend({'Actual $\mathbf{x}(t)$', 'Desired $\mathbf{x}_d(t)$'}, 'Interpreter', 'latex', ...
        'FontSize', legend_font_size, 'Location', 'northeast');
    title('Trajectory 3D', 'Interpreter', 'latex', 'FontSize', title_font_size);
    view(15.6875,37.9459)
    grid on;

    % X, Y, Z Plots
    plot_single_dimension(time, x_hist, xd, '$x$ [m]', '$x(t)$');
    plot_single_dimension(time, y_hist, yd, '$y$ [m]', '$y(t)$');
    plot_single_dimension(time, z_hist, zd, '$z$ [m]', '$z(t)$');

    % Rotor velocities plot
    figure;
    set(gcf,'paperposition',[0 0 8 5],'papersize',[8 5])
    plot(time, rotor_velocities_history(1, :), 'Color', red, 'LineWidth', 1.5); hold on;
    plot(time, rotor_velocities_history(2, :), 'Color', orange,'LineWidth', 1.5);
    plot(time, rotor_velocities_history(3, :), 'Color', blue,'LineWidth', 1.5);
    plot(time, rotor_velocities_history(4, :), 'Color', green, 'LineWidth', 1.5);
    xlabel('$t$ [s]', 'Interpreter', 'latex', 'FontSize', label_font_size);
    ylabel('$\omega_i$ [rad/s]', 'Interpreter', 'latex', 'FontSize', label_font_size);
    legend({'$\omega_1(t)$', '$\omega_2(t)$', '$\omega_3(t)$', '$\omega_4(t)$'}, 'Interpreter', 'latex', 'FontSize', legend_font_size);
    title('Rotor velocities $\omega_i(t)$', 'Interpreter', 'latex', 'FontSize', title_font_size);
    grid on;

    % Rotor forces
    figure;
    set(gcf,'paperposition',[0 0 8 5],'papersize',[8 5])
    plot(time, rotor_forces_history(1, :), 'Color', red, 'LineWidth', 1.5); hold on;
    plot(time, rotor_forces_history(2, :), 'Color', orange,'LineWidth', 1.5);
    plot(time, rotor_forces_history(3, :), 'Color', blue,'LineWidth', 1.5);
    plot(time, rotor_forces_history(4, :), 'Color', green, 'LineWidth', 1.5);
    xlabel('$t$ [s]', 'Interpreter', 'latex', 'FontSize', label_font_size);
    ylabel('$F_i$ [N]', 'Interpreter', 'latex', 'FontSize', label_font_size);
    legend({'$F_1(t)$', '$F_2(t)$', '$F_3(t)$', '$F_4(t)$'}, 'Interpreter', 'latex', 'FontSize', legend_font_size);
    title('Rotor velocities $F_i(t)$', 'Interpreter', 'latex', 'FontSize', title_font_size);
    grid on;
end

function plot_single_dimension(time, actual, desired, label, title_str)
    global title_font_size label_font_size legend_font_size
    global green blue
    % Single dimension plot for X, Y, or Z
    figure;
    set(gcf,'paperposition',[0 0 8 5],'papersize',[8 5])
    plot(time, actual, 'Color', blue, 'LineWidth', 1.5); hold on;
    plot(time, desired, '--','Color', green,'LineWidth', 1.5);
    xlabel('$t$ [s]', 'Interpreter', 'latex', 'FontSize', label_font_size);
    ylabel(label, 'Interpreter', 'latex', 'FontSize', label_font_size);
    legend({'Actual ', 'Desired '}, 'Interpreter', 'latex', 'FontSize', legend_font_size);
    title(title_str, 'Interpreter', 'latex', 'FontSize', title_font_size);
    grid on;
end


