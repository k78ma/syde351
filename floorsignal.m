% Parameters
tile_length = 30.48; % cm
grout_length = 1; % cm
grout_depth = 0.5; % cm
total_length = 500; % cm
profile_resolution = 0.01; % cm (sampling resolution)

% Time and Position Vectors
x = 0:profile_resolution:total_length;
y = zeros(size(x));

% Generate Profile
for i = 1:length(x)
    pos = mod(x(i), tile_length + grout_length);
    if pos <= tile_length
        y(i) = 0; % Tile
    else
        grout_pos = pos - tile_length;
        y(i) = 0 - grout_depth * (1 - cos(2 * pi * grout_pos / grout_length)) / 2; % Grout
    end
end

% Plot the Profile
figure;
plot(x, y, 'LineWidth', 1.1);
title('Vertical Input Profile for Robot Wheels');
xlabel('Position (cm)');
ylabel('Vertical Displacement (cm)');
grid on;

% Save Profile to Workspace
assignin('base', 'x', x);
assignin('base', 'y', y);