t = 0:0.01:10; % Time vector
u = sin(t);    % Data vector

%timescale = linspace(0, t, length(u));
x = timeseries(u, t);

out = sim('testsys.slx', 1000);

test = out.simout(1, : );

figure
plot(test);