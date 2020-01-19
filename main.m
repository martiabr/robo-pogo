% neutral point: position resulting in zero acceleration i.e. constant
% velocity of hopper
% Start with simulation, then add PD control, then try optimal control
% Possibly try more advanced model after? Terrain?

% Constants:
g = 9.81;
r = 1;
m = 1;
l_0 = 1;

% Let y be horizontal distance, z be vertical distance, w be angular
% velocity of body, theta be counter-clockwise leg angle from y axis and l
% be the leg length

% Flight dynamics:
% x_f = [y, y_dot, z, z_dot, theta]
A_f = [0 1 0 0 0;
       0 0 0 0 0;
       0 0 0 1 0;
       0 0 0 0 0;
       0 0 0 0 0];
B_f = [0 0 0 0 1]';
f_f = @(x_f, w) A_f * x_f + B_f * w + [0 0 0 -g 0]';

% Touchdown detector:
td = @(x_f) x_f(3) - l_0 * sin(x_f(5)) <= 0 && x_f(4) < 0;

% Stance dynamics:
% x_s = [theta, theta_dot, l, l_dot]
f_s = @(x_s, u) [x_s(2); -2 * x_s(2) * x_s(4) / x_s(3) - g * cos(x_s(1)) / x_s(3);
            x_s(4); -g * sin(x_s(1)) + x_s(2)^2 * x_s(3) + r / m * (l_0 - x_s(3))] + ...
            [0 0; 0 1 / (m * x_s(3)^2); 0 0; r / m 0] * u;
        
% Takeoff detector:
to = @(x_s, u) cos(x_s(1)) * x_s(2) * x_s(3) + sin(x_s(1)) * x_s(4) > 0 && ...
               r / m * sin(x_s(1)) * (l_0 - x_s(3) + u(1)) + ...
               cos(x_s(1)) * u(2) / (m * x_s(3)) >= 0;

% Simulation initialization:
dt = 0.001;
K = 1000;
t = 0:dt:((K - 1) * dt);
n_f = 5;
n_s = 4;
x_f = zeros(n_f, K);
x_f(:, 1) = [0 0.01 1 0 0]';
s = 0;  % state variable, 0 is flight and 1 is stance

% Simulation:
for k = 1:(K - 1)
    if s
        % Simulate stance dynamics:
    else
        % Simulate flight dynamics:
        x_f(:, k + 1) = x_f(:, k) + f_f(x_f(:, k), 0.1) * dt;
    end
    if td(x_f(:, k + 1))
        break
    end
end

% Plot:
figure(1); clf;
plot(x_f(1, :), x_f(3, :));


