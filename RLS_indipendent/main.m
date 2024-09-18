clear; close all; clc 
addpath("functions");
import Drone.*
import Artva.*
import Plotter.*
import RLS.*
import MagneticField.*

%% CHOSEN Variables
global do_indipendent;
do_indipendent = false;
show_simulation = true;
global add_noise;
add_noise = false;
control_time = 40; % SEMPRE 4  
global threshold;
threshold = 0.020; %--> 2 cm 
global time_step;
time_step = 0.01;
global distributed_estimation_mode;
distributed_estimation_mode = false;
global trajectory_type;
trajectory_type = "circ"; % Either "circ","patrol","rect"
global see_text;
see_text = false;
global drones_num;
drones_num = 12;
desired_goals = [71, 83 ,0; ...
                 50, 50 ,0;];
global n_sources;
n_sources = size(desired_goals,1);
global video_name;
video_name = 'prove';
global do_Video;
do_Video = false;

%% Set seed for reproducibility
seed = 42;
rng(seed)

%% Constants
NONE = -1;
drones_list = NONE;
drones_x_array = zeros(1, drones_num);
drones_y_array = zeros(1, drones_num);
artvas = NONE;
estimates = NONE;
estimates_array = zeros(n_sources, 2);
p = NONE;
time_instant = 0;
global dronesSetted;
dronesSetted = false;
global last_estimate
last_estimate = [-1;-1];
global control_steps;
control_steps = int16(control_time/time_step);
history_var = zeros(2, control_steps);
check = ones(1, control_steps);
k = 1;


%% Magnetic Field
fields_array = cell(n_sources, 1);

% 3D Case
global R_matrices
R_matrices = cell(n_sources, 1);
n_points = 40;
space3D = Space('3D', 3, n_points);
pitch = deg2rad(35); 
for i = 1:n_sources
    R = space3D.RotationMatrix(i, 0, pitch, 0); % Use roll, pitch, and yaw
    R_matrices{i} = R;
    space3D.plotCoordinateSystem(R, desired_goals(i, :),['Frame n', num2str(i)]);
    % Homogenous Transformation
    p_source = desired_goals(i, :)';
    p_local = R.' * (space3D.points - p_source);
    % Calculate the magnetic field
    magnetic_field = MagneticField();
    magnetic_field = magnetic_field.compute(p_local, space3D);
    fields_array{i} = magnetic_field;
    magnetic_field.plot(space3D, n_sources, i, false);
end

% 2D Case
%space2D = Space('2D', 2, n_points);
for i = 1:n_sources
    %space2D.plotCoordinateSystem(R_matrices{i}, desired_goals(i, 1:2),['Frame n', num2str(i)], true);
    magnetic_field = fields_array{i};
    %magnetic_field.plot(space2D, n_sources, i, true);
end

% Initialize the GradientofSum object with magnetic field data and space data
gradient3D = GradientofSum(fields_array, space3D);

% Sum the magnetic field vectors
gradient3D = gradient3D.sumFieldVectors();

% Compute the gradient matrix G
gradient3D = gradient3D.computeGradient();

% Check simmetry
%gradient3D.checkSymmetry();

% Plotting each method with proper figure management
% TA1 has a range of +−atan(sqrt(2)) and TA2 has a range of −π/2 to π/2. They are positive over the source when
% orientation of the magnetic moment is not horizontal.

%methods = { 'Tilt1', 'Tilt2', 'Theta','NSS'};
methods = {'NSS'};
    for i = 1:length(methods)
        figure('Name', [methods{i} ' Edge Detection'], 'NumberTitle', 'off'); % Create a new figure
        values = gradient3D.plotResults(methods{i}); % Plot results for the current method
        drawnow; % Ensure the figure is rendered before proceeding
    end

% Variables specific to Distributed mode
est_artva_x_array = zeros(1, drones_num);
est_artva_y_array = zeros(1, drones_num);
consensus_mean_array = zeros(2,drones_num);
sync_delay = 0.1; % in seconds

%% Init
[drones_list, artvas, estimates] = setup(drones_num, desired_goals);
if(show_simulation)
    p = Plotter();
end

%% Simulation loop
while true
    tic % start timing
    if(~distributed_estimation_mode)
        % For now do the replan only towards the first ARTVA
        if do_indipendent
            drones_list = replan(drones_list, drones_num, estimates{1}.estimated_position);
        elseif ~do_indipendent
            drones_list = replan(drones_list, drones_num, estimates.estimated_position(1:3));
        end

        % Move the drones
        for i = 1:drones_num
            drones_list{i} = drones_list{i}.move();
            drones_x_array(i) = drones_list{i}.position(1);
            drones_y_array(i) = drones_list{i}.position(2);
        end
        % Estimate the position of the ARTVAs
        if do_indipendent
            for j=1:n_sources
                estimates_array(:,j) = estimates{j}.rls(drones_list, artvas{j});
            end
        elseif ~do_indipendent
            estimates_array = estimates.rls(drones_list, artvas);
        end
        % Save the values to check when the algorithm is not updating the values anymore, only the ones of the first ARTVA
        history_var(:,k) = [estimates_array(1,1); 
                            estimates_array(2,1)];
        [result,check,k,history_var] = stopping_criterium(history_var,check,k);
        if result
            disp("Simulation stopped at: " + time_instant + " seconds" );
            disp("The value of the estimate did not change by " + threshold + " m for more than "+ control_time + "s");
            for j=1:n_sources
                display_results(j, estimates_array(:,j), artvas{j}.position');
            end
            break;
        end
    elseif(distributed_estimation_mode)
        drones_list = replan(drones_list, drones_num, [est_artva_x_array; est_artva_y_array; zeros(1,drones_num)]);
        for i = 1:drones_num
            drones_list{i} = drones_list{i}.move();
            drones_x_array(i) = drones_list{i}.position(1);
            drones_y_array(i) = drones_list{i}.position(2);
            if(mod(time_instant, sync_delay) <= 0.01)
                drones_list{i} = drones_list{i}.sync(drones_list);
            end
            drones_list{i} = drones_list{i}.estimate(artvas{1});
            est_artva_x_array(i) = drones_list{i}.est_pos(1);
            est_artva_y_array(i) = drones_list{i}.est_pos(2);
            % Ace filter
            drones_list{i} = drones_list{i}.ace(drones_list);
            consensus_mean_array(:,i) = drones_list{i}.z_new;
        end
        
        history_var(:,k) = consensus_mean_array(:,drones_num); % prendo l'ultimo perchè nella rect è quello che fa più strada 
        if dronesSetted
            [result,check,k,history_var] = stopping_criterium(history_var,check,k); 
            if result
                disp("Simulation stopped at: " + time_instant + " seconds" );
                disp("The value of the estimate did not change by " + threshold + " m for more than "+ control_time + "s");
                %{
                for j=1:n_sources
                    display_results(j, estimates_array(:,j), artvas{j}.position');
                end
                %}
                disp("You have estimated the goal with a TRUE  error of: " + norm((artvas{1}.position(1,1:2)) - (mean([est_artva_x_array; est_artva_y_array],2)')) + "m");
                disp("You have estimated the goal with an error wrt CONSENSUS MEAN: " + norm((artvas{1}.position(1,1:2)) - (consensus_mean_array(:,drones_num)')) + "m");
                disp("Goal position: " + artvas{1}.position);
                disp("Consensus Mean " + consensus_mean_array(:,drones_num)')
                disp("True mean " + mean([est_artva_x_array; est_artva_y_array],2)')
                break;
            end
        end
    end

    %%  This check can be eliminated
    if drones_list{1}.position(2) > 100
        disp(drones_list{1}.position(2))
        disp("Outside boundaries")
        break;
    end

    if(show_simulation)
        if(~distributed_estimation_mode)
            p.draw([drones_x_array; drones_y_array], desired_goals, estimates_array', time_instant);
        else
            p.draw([drones_x_array; drones_y_array], desired_goals, [est_artva_x_array; est_artva_y_array],time_instant,consensus_mean_array);
        end
        elapsed_time = toc;
        pause(max(0,time_step-elapsed_time)) %I take out the computational time so that it acually pauses for only one millisecond 
    end
    time_instant = time_instant + time_step;
end

p.close();

%% Functions
function [result,check,k,history_var] = stopping_criterium(history_var,check,k)
    result = false;
    global threshold;
    global control_steps;
    global last_estimate;
    %sum(check(1,:),'all')
    % If for consecutives times the check is always true then stop the simulation
    if sum(check(1,:),'all') == 0 && (norm(history_var(:,1)' - history_var(:,control_steps)') < threshold)
        result = true;
    end
    if k > 1
        if norm(history_var(:,k)' - history_var(:,k-1)') < threshold
            check(1,k) = 0;
        else 
            check(1,k) = 1;
        end
    end
    if k == 1
        if norm(history_var(:,1)' - last_estimate') < threshold
            check(1,1) = 0;
        else 
            check(1,1) = 1;
        end
    end
    if k == control_steps
        last_estimate = history_var(:,k);
        if norm(history_var(:,k)' - history_var(:,k-1)') < threshold
            check(1,k) = 0;
        else 
            check(1,k) = 1;
        end
        k = 0;
        history_var = zeros(2,control_steps);
    end
    k = k + 1;
end

function display_results(i, estimation, truth)
    disp("RESULTS FOR ARTVA " + i);
    disp("You have estimated the goal with an error of: " + norm((estimation) - (truth(1:2))) + " m");
    fprintf('Goal position: [%.3f, %.3f, %.3f]\n', truth);
    fprintf('Estimation: [%.3f, %.3f, %.3f]\n', estimation);
end
