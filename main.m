 %% Global Parameters
base_mass = 20; %kg
load_mass = 20; %kg
total_mass = base_mass + load_mass; %kg
voltage_left = 0;
voltage_right = 0;
load_torque = 0;

%% Motor Parameters
La = 0.15*10^-3;
Ra = 0.035;
Kt = 8*10^-3;
Kb = 8*10^-3;
If = 1.15;
Cshaft = 1;

%% Suspension Parameters
c = 10;
k = 10;
speed = 0.508; %m/s

distance_between_wheels = 0.5; %m
wheel_radius = 0.01; %m

input_torque = 0; % No additional load other than mechanical resistance.
gear_ratio = 3;

%% Floor parameters
%note - these values are actually the distance travelled
Dtile = 0.3048;
Dgroove = 0.01;
Hgroove = 0.005;

%% Door parameters
%note - these values are actually the distance travelled
Ddoor = 0.05;
Hdoor = 0.03;
Ddelay = 0;

%% Robot Motion Parameters (Tune the robot motion here)
% %Tune the parameters found in the path generation functions at the end of
%this file
%floorsig = sim("floorsignal.slx", 5/speed);
%doorsig = sim("DoorSignal.slx", 5/speed + 0.2);

%sim("Suspension_System.slx", 5/speed + 0.3);




%% Path Planning
t = 0;
% 15m Forward
[v_l, v_r, t_new] = path_linear(15);
v_left = v_l;
v_right = v_r;
t = t + t_new;

% 90 deg CW
[v_l, v_r, t_new] = path_rot(90, true);
% Accelerate
v_left = [v_left, v_l];
v_right = [v_right, v_r];
t = t + t_new;

% 10m Foreward
[v_l, v_r, t_new] = path_linear(10);
v_left = [v_left, v_l];
v_right = [v_right, v_r];
t = t + t_new;

% 90 deg CW
[v_l, v_r, t_new] = path_rot(90, true);
v_left = [v_left, v_l];
v_right = [v_right, v_r];
t = t + t_new;

% 8m Foreward
[v_l, v_r, t_new] = path_linear(8);
v_left = [v_left, v_l];
v_right = [v_right, v_r];
t = t + t_new;

% Generate total time and timeseries:
timescale = linspace(0, t, length(v_right));

% figure
% plot(v_left)
% figure
% plot(v_right)

left_motor_voltage = timeseries(v_left, timescale);
right_motor_voltage = timeseries(v_right, timescale);

%path = sim("WheelDriveModel.slx", 1000);

tiles = tilesignal(33);
door = doorsignal(10);
door = [door, zeros(1, length(tiles) - length(door))];
floor_signal = door + tiles;

robot_displacement = sim("suspensionsignal.slx").signal;

figure;
plot(floor_signal)

function [v_path_left, v_path_right, path_time] = path_linear(distance)
    max_acceleration = 3;
    drive_voltage = 24;
    drive_speed = 3;

    resolution = 100;
    
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
    max_turning_acceleration = 15; % deg/s
    turning_velocity = 25; % deg/s
    turning_voltage = 5;

    resolution = 100;

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
