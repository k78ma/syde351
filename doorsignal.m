function y = doorsignal(distanceToDoor)
    % Parameters
    threshold_length = 5; % cm
    threshold_height = 3; % cm
    total_length = distanceToDoor * 100; % cm
    profile_resolution = 0.01;
    
    % Time and Position Vectors
    x = 0:profile_resolution:total_length;
    y = zeros(size(x));
    
    % Generate Profile
    start_pos = total_length - threshold_length; % Starting position of the threshold
    for i = 1:length(x)
        if x(i) >= start_pos
            pos = x(i) - start_pos;
            y(i) = threshold_height * (1 - cos(2 * pi * pos / threshold_length)) / 2;
        else
            y(i) = 0; % Flat section
        end
    end
end



% % Plot the Profile
% figure;
% plot(x, y);
% title('Haversine Cross-Section Profile for Door Threshold at the End');
% xlabel('Position (cm)');
% ylabel('Height (cm)');
% grid on;
% 
% % Save Profile to Workspace
% assignin('base', 'x', x);
% assignin('base', 'y', y);
