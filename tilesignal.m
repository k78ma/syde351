function y = tilesignal(distance)
    % Parameters
    tile_length = 30.48; % cm
    grout_length = 1; % cm
    grout_depth = 0.5; % cm
    total_length = distance * 100; % cm
    profile_resolution = 0.01; % cm
    
    x = 0:profile_resolution:total_length;
    y = zeros(size(x));
    
    for i = 1:length(x)
        pos = mod(x(i), tile_length + grout_length);
        if pos <= tile_length
            y(i) = 0; % Tile
        else
            grout_pos = pos - tile_length;
            y(i) = 0 - grout_depth * (1 - cos(2 * pi * grout_pos / grout_length)) / 2;
        end
    end
end


% % Plot the Profile
% figure;
% 
% title('Vertical Input Profile for Robot Wheels');
% xlabel('Position (cm)');
% ylabel('Vertical Displacement (cm)');
% grid on;
% 
% % Save Profile to Workspace
% assignin('base', 'x', x);
% assignin('base', 'y', y);