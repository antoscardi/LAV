classdef Controller < handle
    % Drone controller class
    properties
        % Drone parameters
        m = 0.4;                                   % Mass of the drone (kg)                         
        Ix =  3.8278e-3; Iy = 3.8278e-3; Iz = 7.1345e-3;
        l = 0.205;                                 % Length arms (m), distance from center to propellers
        c_t =  6.354e-4;                            % Force/Thrust coefficient
        c_d = 0.048;                               % Torque coefficient (DRAG/MOMENT)
        J;                                         % Inertia matrix
        g = 9.81;                                 % Gravity (m/s^2)
        c;  
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
            obj.c = obj.c_d/obj.c_t;  
        end

        function x_dot = quadrotor_full_dynamics(obj, dt, state, F, input_torques)
            v = state(4:6); phi = state(7); theta = state(8); psi = state(9);
            p = state(10); q = state(11); r = state(12);
            
            % --- TRANSLATIONAL DYNAMICS ---
            vx_dot =  F / obj.m * (cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi));
            vy_dot =  F / obj.m * (cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi));
            vz_dot =  F / obj.m * (cos(phi)*cos(theta)) - obj.g;
        
            % --- ROTATIONAL DYNAMICS ---
            p_dot = q*r*(obj.Iy-obj.Iz)/obj.Ix + input_torques(1)/obj.Ix;
            q_dot = p*r*(obj.Iz-obj.Ix)/obj.Iy + input_torques(2)/obj.Iy;
            r_dot = p*q*(obj.Ix-obj.Iy)/obj.Iz + input_torques(3)/obj.Iz;
            phi_dot = p + sin(phi) * tan(theta) *q + cos(phi) * tan(theta) *r;
            theta_dot = cos(phi) * q - sin(phi)* r;
            psi_dot = sin(phi) * sec(theta) * q + cos(phi) * sec(theta) * r;
            
            x_dot = [v, vx_dot, vy_dot, vz_dot, phi_dot, theta_dot, psi_dot, p_dot, q_dot, r_dot];
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

        function rotor_velocities = compute_rotor_velocities(obj, F, torques) 
            % Solve for rotor speeds squared
            A = obj.c_t * [
                1, 1, 1, 1;              % Total thrust
                0, -obj.l, 0, obj.l;             % Roll torque
                obj.l, 0, -obj.l, 0;             % Pitch torque
                -obj.c, +obj.c, -obj.c, +obj.c;            % Yaw torque
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
    end
end


