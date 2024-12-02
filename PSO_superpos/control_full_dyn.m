clear; close all; clc;

% Parameters
global Ix Iy Iz g m J o
mass = 1;                                                       % Mass of the drone (kg)
g = 9.81;                                                       % Gravity (m/s^2)
Ix = 0.0820; Iy = 0.0845; Iz = 0.1377; J = diag([Ix, Iy, Iz]);  % Inertia matrix (kg·m^2)
T = 3;                                                          % (Planning) period T
m = 4.34;
o = 2 * pi / T;                                                 % Frequency for circular trajectory

% Controller gains
global c0 c1 c2 c3
c3 = 4.8;    % Damping term for higher stability
c2 = 10.8;   % Proportional term for fast tracking
c1 = 11.2;  % Higher-order term to prevent oscillations
c0 = 4;  % Integral term for steady-state accuracy

% Simulation setup
dt = 0.01; % Time step
Tspan = 0:dt:10; % Simulation time
N = length(Tspan); % Number of steps

% Initial states
state = zeros(14, 1);     % Initialize state vector
state(1:3) = [0 0 0]';    % Initial position
state(4:6) = [0 0 0]';    % Initial velocity
state(7:9) = [0 0 0]';    % Initial roll, pitch, yaw
state(10:12) = [0 0 0]';  % Initial angular velocity
state(13) = 0.1;          % Initial force
state(14) = 0;            % Initial force derivative

% Trajectory setup
xd = cos(o * Tspan);
yd = sin(o * Tspan);
zd = zeros(size(Tspan)); % Flat trajectory for simplicity
yawd = zeros(size(Tspan));

% Storage for results
states = zeros(14, N);

% Online simulation loop
for k = 1:N
    t = Tspan(k);

    % Save state for analysis
    states(:, k) = state;

    % Linear controller
    v = linear_controller(t, state);

    % Dynamic compensator
    utilde = dynamic_compensator(state, v);

    % Extract forces and moments
    u = [state(13); utilde(2:4)];

    % Update states using quadrotor dynamics
    dquadrotor_model = quadrotor_model(state, u);

    % Combine all derivatives
    dzeta = [state(14); utilde(1)]; 
    dstate = [dquadrotor_model; dzeta];

    % Euler integration for online update
    state = state + dt * dstate;
end

% Extract results
x = states(1, :);
y = states(2, :);
z = states(3, :);
yaw = states(9, :);

% Plot results
figure(1); plot(Tspan, x, Tspan, xd); legend('x', 'x_d'); xlabel('t [sec]'); ylabel('x [m]'); title('Position: x(t) and x_d(t)');
figure(2); plot(Tspan, y, Tspan, yd); legend('y', 'y_d'); xlabel('t [sec]'); ylabel('y [m]'); title('Position: y(t) and y_d(t)');
figure(3); plot(Tspan, z, Tspan, zd); legend('z', 'z_d'); xlabel('t [sec]'); ylabel('z [m]'); title('Position: z(t) and z_d(t)');
figure(4); plot(Tspan, yaw, Tspan, yawd); legend('yaw', 'yaw_d'); xlabel('t [sec]'); ylabel('yaw [rad]'); title('Yaw: yaw(t) and yaw_d(t)');

% Functions remain unchanged: quadrotor_model, linear_controller, dynamic_compensator

function dquadrotor_model = quadrotor_model(state, u)
    %% Initializing variables
    v = state(4:6); phi = state(7); theta = state(8); psi = state(9); omega = state(10:12);
    global m g J
    cphi = cos(phi); sphi = sin(phi); ctheta = cos(theta); stheta = sin(theta); cpsi = cos(psi); spsi = sin(psi); ttheta = tan(theta);
    f = u(1); M = u(2:4);

    %% Velocity
    dv = -f / m * [cphi * stheta * cpsi + sphi * spsi;
                   cphi * stheta * spsi - sphi * cpsi;
                   cphi * ctheta];
    dv(3) = dv(3) + g;

    %% RPY
    drpy = [omega(1) + sphi * ttheta * omega(2) + cphi * ttheta * omega(3);
            cphi * omega(2) - sphi * omega(3);
            sphi / ctheta * omega(2) + cphi / ctheta * omega(3)];

    %% Angular velocity
    domega = inv(J)\(M - cross(omega, J * omega));

    %% Output
    dquadrotor_model = [v; dv; drpy; domega];
end


function out = full_dynamics_ode(t,state)

    %% linear controller
    v = linear_controller(t,state);
    
    %% dynamic compensator
    utilde = dynamic_compensator(state,v);
    dzeta = [state(14);utilde(1)];
    
    %% quadrotor
    u = [state(13);utilde(2:4)];
    dquadrotor_model = quadrotor_model(state,u);
    
    % ode output
    out = [dquadrotor_model;dzeta];
    
    end


function v = linear_controller(t, state)
    %% initializing variables 
    x = state(1:3); x_dot = state(4:6);
    phi = state(7); theta = state(8); psi = state(9);
    p = state(10); q = state(11); r = state(12);
    omega = state(10:12);
    f = state(13); f_dot = state(14);
    
    global m g J
    cphi = cos(phi); sphi = sin(phi); ctheta = cos(theta); stheta = sin(theta); cpsi = cos(psi); spsi = sin(psi); ttheta = tan(theta);
    Jinv = inv(J);
    J1X = Jinv(1,:); J2X = Jinv(2,:); J3X = Jinv(3,:);
    cross_omega_Jomega = cross(omega,J*omega);
    
    global c0 c1 c2 c3 o

    %%%%%%%%%%%%%%%%%%% circumference %%%%%%%%%%%%%%%%%
    xd      = [cos(o*t)         sin(o*t)        0           ]';
    xd_dot  = [-o*sin(o*t)      o*cos(o*t)      0           ]';
    xd_2dot = [-o^2*cos(o*t)    -o^2*sin(o*t)   0           ]';
    xd_3dot = [o^3*sin(o*t)     -o^3*cos(o*t)   0           ]';
    xd_4dot = [o^4*cos(o*t)     o^4*sin(o*t)    0           ]';
    yawd        = 0;
    yawd_dot    = 0;
    yawd_2dot   = 0;
    
    % %%%Just Go high%%%%%%%%%%%%%%%
    % xd= [0         0        3]';
    % xd_dot= [0         0        0]';
    % xd_2dot= [0         0        0]';
    % xd_3dot= [0         0        0]';
    % xd_4dot= [0         0        0]';
    % yawd        = 0;
    % yawd_dot    = 0;
    % yawd_2dot   = 0;
    
    %% time derivative of rpy
    phi_dot = p+sphi*ttheta*q+cphi*ttheta*r;
    theta_dot = cphi*q-sphi*r;
    psi_dot = sphi/ctheta*q+cphi/ctheta*r;
    
    %% computing derivatives
    
    x_2dot = -f/m*[ cpsi*stheta*cphi+spsi*sphi;
                    spsi*stheta*cphi-sphi*cpsi;
                    ctheta*cphi] + [0 0 g]';
    
    x_3dot = -f/m*[-sphi*stheta*cpsi*phi_dot+cphi*ctheta*cpsi*theta_dot-cphi*stheta*spsi*psi_dot+...
                    cphi*spsi*phi_dot + sphi*cpsi*psi_dot;
                    -sphi*stheta*spsi*phi_dot+cphi*ctheta*spsi*theta_dot+cphi*stheta*cpsi*psi_dot+...
                    -cphi*cpsi*phi_dot+sphi*spsi*psi_dot;
                    -sphi*ctheta*phi_dot-cphi*stheta*theta_dot]+...
             -f_dot/m*[cpsi*stheta*sphi+sphi*spsi;
                       cphi*stheta*spsi-sphi*cpsi;
                       ctheta*cphi];
    
    yaw = psi;
    yaw_dot = psi_dot;
    
    %% errors
    e0 = xd-x;
    e1 = xd_dot-x_dot;
    e2 = xd_2dot-x_2dot;
    e3 = xd_3dot-x_3dot;
    
    e1yaw   = yawd_dot-yaw_dot;
    e0yaw   = yawd-yaw;
    
    %% output control law
    vx = xd_4dot + c3*e3 + c2*e2 + c1*e1 + c0*e0;
    vyaw = yawd_2dot + c1*e1yaw + c0*e0yaw; 
    v = [vx;vyaw];
    
end


function utilde = dynamic_compensator(state,v)

    %% initializing variables
    
    phi = state(7); theta = state(8); psi = state(9);
    phidot = state(10); thetadot = state(11); psidot = state(12);
    
    global m g J
    Ix = J(1,1); Iy = J(2,2); Iz = J(3,3);
    
    T = state(13);
    Tdot = state(14);
    
    %% Regular state feedback
    sph=sin(phi);cph=cos(phi);st=sin(theta);ct=cos(theta);sps=sin(psi);cps=cos(psi); 
    
    Jinv = [-m*(sph*sps+cph*cps*st)        m*(cps*sph-cph*sps*st)         -m*cph*ct       0;
             -m*Ix/T*(cph*sps-cps*sph*st)   m*Ix/T*(cph*cps+sph*sps*st)   m*Ix/T*ct*sph   Ix*st;
             -m*Iy/T*cps*ct/cph             -m*Iy/T*ct*sps/cph            m*Iy/T*st/cph   -Iy*ct*tan(phi);
             0                              0                             0               Iz              ];         
    
    l1 = -2*Tdot/m*((cps*sph-sps*st*cph)*psidot+(cps*ct*cph)*thetadot+(sps*cph-cps*st*sph)*phidot)-...
         T/m*((-sps*sph*psidot+cps*cph*phidot-cps*st*cph*psidot-sps*ct*cph*thetadot+sps*st*sph*psidot)*psidot+...
         (-sps*ct*cph*psidot-cps*st*cph*thetadot-cps*ct*sph*phidot)*thetadot+...
         (cps*cph*psidot-sps*sph*phidot+sps*st*sph*psidot-cps*ct*sph*thetadot-cps*st*cph*phidot)*phidot);
     
    l2 = -2*Tdot/m*((cps*st*cph+sph*sps)*psidot+(sps*ct*cph)*thetadot+(-sps*st*sph-cph*cps)*phidot)-...
         T/m*((-sps*st*cph*psidot+cps*ct*cph*thetadot-cps*st*sph*phidot+cph*sps*phidot+sph*cps*psidot)*psidot+...
         (cps*ct*cph*psidot-sps*st*cph*thetadot-sps*ct*sph*phidot)*thetadot+...
         (-cps*st*sph*psidot-sps*ct*sph*thetadot-sps*st*cph*phidot+sph*cps*phidot+cph*sps*psidot)*phidot);
     
    l3 = -2*Tdot/m*(-st*cph*thetadot-ct*sph*phidot)-...
         T/m*((-ct*cph*thetadot+st*sph*phidot)*thetadot+(st*sph*thetadot-ct*cph*phidot)*phidot);
     
    l4 = 0;
    
    l = [l1 l2 l3 l4]';
    
    utilde = Jinv*(-l+v);
       
end