classdef Controller < handle
    % Drone controller class
    properties
        % Drone parameters
        m = 4;                                   % Mass of the drone (kg)
        g = 9.81;                                % Gravity (m/s^2)
        Ix = 0.5; 
        Iy = 0.5; 
        Iz = 0.9; 
        J;                                       % Inertia matrix
        % Controller gains
        kxp = 0.1; kxd = 0.5;
        kyp = 0.1; kyd = 0.54;
        kzp = 1; kzd = 2.5;
        kp_phi = 2;  kd_phi = 2.5;
        kp_theta = 2;  kd_theta = 2.5;
        kp_psi = 2;  kd_psi = 4;
    end

    methods
        function obj = Controller()
            % Constructor to initialize properties
            obj.J = diag([obj.Ix, obj.Iy, obj.Iz]);
        end

        function x_dot = simplified_model(obj, dt, state, F, input_torques)
            % Simplified drone dynamics model

            % Extract state variables
            v = state(4:6); 
            phi = state(7); 
            theta = state(8); 
            psi = state(9); 
            omega = state(10:12);

            % --- TRANSLATIONAL DYNAMICS ---
            vx_dot =  F / obj.m * (cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi));
            vy_dot =  F / obj.m * (cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi));
            vz_dot =  F / obj.m * (cos(phi)*cos(theta)) - obj.g;

            % --- ROTATIONAL DYNAMICS ---
            p_dot = input_torques(1) / obj.Ix;
            q_dot = input_torques(2) / obj.Iy;
            r_dot = input_torques(3) / obj.Iz;
            rpy_dot = omega;

            % State derivative
            x_dot = [v, vx_dot, vy_dot, vz_dot, rpy_dot, p_dot, q_dot, r_dot];
        end

        function [input_torques, input_force] = control_laws(obj, state, desired_position, desired_velocity, desired_acceleration)
            % Control laws for position and attitude control

            % Extract state variables
            position = state(1:3);
            velocity = state(4:6);
            phi = state(7); 
            theta = state(8); 
            psi = state(9);
            phi_dot = state(10); 
            theta_dot = state(11); 
            psi_dot = state(12);

            % --- Position Control (Outer Loop) ---
            % Compute position and velocity errors
            pos_error = desired_position - position;
            vel_error = desired_velocity - velocity;

            ex = pos_error(1); 
            ey = pos_error(2); 
            ez = pos_error(3);
            ex_dot = vel_error(1); 
            ey_dot = vel_error(2); 
            ez_dot = vel_error(3);

            % HEIGHT CONTROL
            input_force = obj.m / max(1e-6, (cos(phi) * cos(theta))) * (obj.g + obj.kzp * ez + obj.kzd * ez_dot);
            if input_force < 0 || isnan(input_force)
                error('Invalid input_force value: %f', input_force);
            end

            % POSITION CONTROL via PD
            % Compute desired thrust direction
            Fx = (obj.m / max(1e-6, input_force)) * (obj.kxp * ex + obj.kxd * ex_dot + desired_acceleration(1));
            Fy = (obj.m / max(1e-6, input_force)) * (obj.kyp * ey + obj.kyd * ey_dot + desired_acceleration(2));

            % Compute desired roll (phi) and pitch (theta) from thrust direction
            phi_d = asin(Fx * sin(psi) - Fy * cos(psi));
            theta_d = asin(Fx * cos(psi) + Fy * sin(psi)) / max(1e-6, cos(phi_d));    
            psi_d = atan2(desired_velocity(2), desired_velocity(1));

            % Compute torques for attitude control
            tau_phi = obj.Ix * (obj.kp_phi * (phi_d - phi) + obj.kd_phi * (-phi_dot));
            tau_theta = obj.Iy * (obj.kp_theta * (theta_d - theta) + obj.kd_theta * (-theta_dot));
            tau_psi = obj.Iz * (obj.kp_psi * (psi_d - psi) + obj.kd_psi * (-psi_dot));

            % Combine torques into a vector
            input_torques = [tau_phi; tau_theta; tau_psi];
        end
    end
end


