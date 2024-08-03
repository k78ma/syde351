close all;
clear;

%% Global Parameters
base_mass = 20; %kg
load_mass = 20; %kg
total_mass = base_mass + load_mass; %kg
load_torque = 1;

%% Motor Parameters
La = 0.15*10^-3;
Ra = 0.035;
K_T = 8*10^-3;
K_B = 8*10^-3;
If = 1.15;
Cshaft = 1;
J = 4;

%% Suspension Parameters
c = 20;
k = 40;
speed = 300; %cm/s

distance_between_wheels = 0.5; %m
wheel_radius = 0.01; %m

input_torque = 0; % No additional load other than mechanical resistance.
gear_ratio = 3;


%% Path Planning
t = 0;
% 15m Forward
[v_l, v_r, t_new] = path_linear(1500, speed);
v_left = v_l;
v_right = v_r;
t = t + t_new;

% 90 deg CW
% [v_l, v_r, t_new] = path_rot(90, true);
% % Accelerate
% v_left = [v_left, v_l];
% v_right = [v_right, v_r];
% t = t + t_new;

% 10m Foreward
[v_l, v_r, t_new] = path_linear(1000, speed);
v_left = [v_left, v_l];
v_right = [v_right, v_r];
t = t + t_new;

% 90 deg CW
% [v_l, v_r, t_new] = path_rot(90, true);
% v_left = [v_left, v_l];
% v_right = [v_right, v_r];
% t = t + t_new;

% 8m Foreward
[v_l, v_r, t_new] = path_linear(800, speed);
v_left = [v_left, v_l];
v_right = [v_right, v_r];
t = t + t_new;

% Generate total time and timeseries:

% 
% figure;
% plot(v_left);
% figure;
% plot(v_right);

% load_torque = linspace(0, 0, length(v_right));
% gear_ratio = linspace(3, 3, length(v_right));
timescale = linspace(0, t, length(v_right));
voltage_right = timeseries(v_right, timescale);
voltage_left = timeseries(v_left, timescale);
model_name = "navigation";

open_system(model_name);
set_param(model_name, 'StopTime', num2str(t));
set_param(model_name, 'Solver', 'ode45'); % You can choose 'ode45', 'ode23', 'ode15s', etc.
set_param(model_name, 'RelTol', '1e-12'); % Adjust this value as needed
set_param(model_name, 'AbsTol', '1e-12'); % Adjust this value as needed
set_param(model_name, 'MaxStep', '0.00005'); % Adjust this value as needed
test = sim(model_name);

% Create a new figure
% Create a new figure
% Create a new figure
figure;

% Plot acceleration
subplot(4, 2, 1); % 4x2 grid, first subplot
plot(test.acceleration.Time, test.acceleration.Data);
title('Acceleration');
xlabel('Time');
ylabel('Acceleration');

% Plot jerk
subplot(4, 2, 2); % 4x2 grid, second subplot
plot(test.jerk.Time, test.jerk.Data);
title('Jerk');
xlabel('Time');
ylabel('Jerk');

% Plot x position
subplot(4, 2, 3); % 4x2 grid, third subplot
plot(test.pos_x.Time, test.pos_x.Data);
title('X Position');
xlabel('Time');
ylabel('X Position');

% Plot y position
subplot(4, 2, 4); % 4x2 grid, fourth subplot
plot(test.pos_y.Time, test.pos_y.Data);
title('Y Position');
xlabel('Time');
ylabel('Y Position');

% Plot voltage left
subplot(4, 2, 5); % 4x2 grid, fifth subplot
plot(timescale, v_left);
title('Voltage Left');
xlabel('Time');
ylabel('Voltage Left');

% Plot voltage right
subplot(4, 2, 6); % 4x2 grid, sixth subplot
plot(timescale, v_right);
title('Voltage Right');
xlabel('Time');
ylabel('Voltage Right');

% Plot velocity
subplot(4, 2, 7); % 4x2 grid, seventh subplot
plot(test.vel.Time, test.vel.Data);
title('Velocity');
xlabel('Time');
ylabel('Velocity');

% Adjust layout
sgtitle('Timeseries and Voltage Plots'); % Optional: add a title for the entire figure



tiles = tilesignal(33);
door = doorsignal(10);
door = [door, zeros(1, length(tiles) - length(door))];
floor_signal = door + tiles;

timescale = linspace(0, t, length(floor_signal));
floor_signal_time = timeseries(floor_signal, timescale);

% model_name = "suspensionsignal2";
% 
% open_system(model_name);
% set_param(model_name, 'StopTime', num2str(t));
% set_param(model_name, 'Solver', 'ode45'); % You can choose 'ode45', 'ode23', 'ode15s', etc.
% set_param(model_name, 'RelTol', '1e-12'); % Adjust this value as needed
% set_param(model_name, 'AbsTol', '1e-12'); % Adjust this value as needed
% set_param(model_name, 'MaxStep', '0.00005'); % Adjust this value as needed
% robot_displacement = sim(model_name).signal;
% 
% % Extract time and data
% time = robot_displacement.Time; % [148x1 double]
% data = squeeze(robot_displacement.Data); % [1x1x148 double] -> [1x148] after squeeze
% 
% fprintf('c: %f, k: %f, displacement: %f, speed: %f\n', c, k, max(abs(data)), speed);
% 
% % Create the plot
% figure; % Opens a new figure window
% plot(time, data); % Plot time vs. data
% xlabel('Time'); % Label for the x-axis
% ylabel('Data'); % Label for the y-axis
% title('Time vs Data'); % Title of the plot
% grid on; % Add grid for better readability
% 
% figure;
% title('Floor Signal');
% plot(timescale, floor_signal);

% figure;
% plot(robot_displacement);
% title('Robot Displacement');
% xlabel('Sample size (n)');
% ylabel('Height Displacement');
% axis([0 3300 -1 1]);

function [v_path_left, v_path_right, path_time] = path_linear(distance, drive_speed)
    max_acceleration = 150;
    drive_voltage = 24;

    resolution = 10000;
    
    acceleration_time = drive_speed/max_acceleration;
    acceleration_distance = 0.5 * max_acceleration * acceleration_time^2;

    % Accel
    v_path_left = linspace(0, drive_voltage, acceleration_time * resolution);
    v_path_right = linspace(0, drive_voltage, acceleration_time * resolution);
    
    % Hold
    distance_full_speed = distance - 2 * acceleration_distance;
    time_full_speed = distance_full_speed / drive_speed;
    v_path_left = [v_path_left, linspace(drive_voltage, drive_voltage, time_full_speed * resolution)];
    v_path_right = [v_path_right, linspace(drive_voltage, drive_voltage, time_full_speed * resolution)];
    
    % Decelerate
    v_path_left = [v_path_left, linspace(drive_voltage, 0, acceleration_time * resolution)];
    v_path_right = [v_path_right, linspace(drive_voltage, 0, acceleration_time * resolution)];
    path_time = 2 * acceleration_time + time_full_speed;
end

function [v_path_left, v_path_right, turn_time] = path_rot(angle, CW)
    max_turning_acceleration = 12.5; % deg/s
    turning_velocity = 25; % deg/s
    turning_voltage = 5;

    resolution = 10000;

    if CW
        right_voltage = -turning_voltage;
        left_voltage = turning_voltage;
    else 
        right_voltage = turning_voltage;
        left_voltage = -turning_voltage; 
    end

    acceleration_time = turning_velocity/max_turning_acceleration;
    acceleration_angle = 0.5 * max_turning_acceleration * acceleration_time^2;

    % acel
    v_path_left = linspace(0, left_voltage, acceleration_time * resolution);
    v_path_right = linspace(0, right_voltage, acceleration_time * resolution);

    % Hold
    dist_full_speed = angle - 2 * acceleration_angle;
    time_full_speed = dist_full_speed / turning_velocity;
    v_path_left = [v_path_left, linspace(left_voltage, left_voltage, time_full_speed * resolution)];
    v_path_right = [v_path_right, linspace(right_voltage, right_voltage, time_full_speed * resolution)];

    % Decelerate
    v_path_left = [v_path_left, linspace(left_voltage, 0,  acceleration_time * resolution)];
    v_path_right = [v_path_right, linspace(right_voltage, 0, acceleration_time * resolution)];

    turn_time = 2 * acceleration_time + time_full_speed;
end
