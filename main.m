% neutral point: position resulting in zero acceleration i.e. constant
% velocity of hopper
% Start with simulation, then add PD control, then try optimal control
% Possibly try more advanced model after? Terrain?

% Constants:
g = 9.81;  % gravity
r = 2000;  % spring constant
m = 10;  % mass
l_0 = 1;  % equilibrium leg length

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

% Stance dynamics:
% x_s = [theta, theta_dot, l, l_dot]
f_s = @(x_s, u) [x_s(2); -2 * x_s(2) * x_s(4) / x_s(3) - g * cos(x_s(1)) / x_s(3);
            x_s(4); -g * sin(x_s(1)) + x_s(2)^2 * x_s(3) + r / m * (l_0 - x_s(3))] + ...
            [0 0; 0 1 / (m * x_s(3)^2); 0 0; r / m 0] * u;
        
% Touchdown detector:
td = @(x_f, x_fo) ~isequal(sign(x_f(3) - l_0 * sin(x_f(5))), ...
                  sign(x_fo(3) - l_0 * sin(x_fo(5)))) && x_f(4) < 0;

% Takeoff detector:
to = @(x_s, x_so, u) cos(x_s(1)) * x_s(2) * x_s(3) + sin(x_s(1)) * x_s(4) > 0 && ...
                     ~isequal(sign(r / m * sin(x_s(1)) * (l_0 - x_s(3) + u(1)) + ...
                     cos(x_s(1)) * u(2) / (m * x_s(3))), ...
                     sign(r / m * sin(x_so(1)) * (l_0 - x_so(3) + u(1)) + ...
                     cos(x_so(1)) * u(2) / (m * x_so(3))));
           
% Flight to stance coordinate transformation (y_f is stance position):
f2s = @(x_f, y_f) [x_f(5);
              (x_f(1) * x_f(4) - x_f(2) * x_f(3)) / ((x_f(1) - y_f)^2 + x_f(3)^2);
              sqrt((x_f(1) - y_f)^2 + x_f(3)^2);
              (x_f(1) * x_f(2) + x_f(3) * x_f(4)) / sqrt((x_f(1) - y_f)^2 + x_f(3)^2)];

% Stance to flight coordinate transformation:
s2f = @(x_s, y_f) [cos(x_s(1)) * x_s(3) + y_f;
             -sin(x_s(1)) * x_s(2) * x_s(3) + cos(x_s(1)) * x_s(4);
              sin(x_s(1)) * x_s(3);
              cos(x_s(1)) * x_s(2) * x_s(3) + sin(x_s(1)) * x_s(4);
              x_s(1)]; 

% Simulation initialization:
dt = 0.0001;
K = 50000;
t = 0:dt:((K - 1) * dt);
s = zeros(1, K);  % state variable, 0 is flight and 1 is stance
s(1) = 0;
n_f = 5;
n_s = 4;
x_f = zeros(n_f, K);
x_f(:, 1) = [l_0 * cos(1.55) 0 2 0 1.55]';
y_f = zeros(1, K);
x_s = zeros(n_s, K);
%x_s(:, 1) = [1.57 -0.2 0.6 0]';
u = [0 1.5]';
w = 0;

% Simulation:
for k = 1:(K - 1)
    if x_f(3, k) < 0
        break
    end
    if s(k)
        % Simulate stance dynamics:
        x_s(:, k + 1) = x_s(:, k) + f_s(x_s(:, k), u) * dt;
        
        y_f(k + 1) = y_f(k);
        
        x_f(:, k + 1) = s2f(x_s(:, k + 1), y_f(k + 1));
       
        if k > 1 && to(x_s(:, k + 1), x_s(:, k), u)  % detect takeoff
            s(k + 1) = 0;
            w = x_s(2, k + 1);  % set new angular velocity
        else
            s(k + 1) = 1;
        end
    else
        % Simulate flight dynamics:
        x_f(:, k + 1) = x_f(:, k) + f_f(x_f(:, k), w) * dt;
        
        y_f(k + 1) = x_f(1, k + 1) - cos(x_f(5, k + 1)) * l_0; % set new stance position
        
        x_s(:, k + 1) = f2s(x_f(:, k + 1), y_f(k + 1));
        x_s(2, k + 1) = w;
        x_s(3, k + 1) = l_0;
        
        if k > 1 && td(x_f(:, k + 1), x_f(:, k))  % detect touchdown
            s(k + 1) = 1;
        else
            s(k + 1) = 0;
        end
    end
end

% Plot:
figure(1); clf;
plot(t, r / m * sin(x_s(1, :)) .* (l_0 - x_s(3, :) + u(1)) + ...
               cos(x_s(1, :)) .* u(2) / (m * x_s(3, :)));  % horizontal acceleration = 0
hold on;
grid on;
plot(t, cos(x_s(1, :)) .* x_s(2, :) .* x_s(3, :) + sin(x_s(1, :)) .* x_s(4, :));  % vertical velocity > 0

figure(2); clf;
subplot(2, 4, 1);
plot(t, x_f(1, :));
title('y');
subplot(2, 4, 2);
plot(t, x_f(2, :));
title('y dot');
subplot(2, 4, 3);
plot(t, x_f(3, :));
title('z');
subplot(2, 4, 4);
plot(t, x_f(4, :));
title('z dot');
subplot(2, 4, 5);
plot(t, x_f(5, :));
title('theta');
subplot(2, 4, 6);
plot(t, x_s(2, :));
title('theta dot');
subplot(2, 4, 7);
plot(t, x_s(3, :));
title('l');
subplot(2, 4, 8);
plot(t, x_s(4, :));
title('l dot');

figure(4); clf;
plot(t, s);

figure(5); clf;
plot(x_f(1, :), x_f(3, :));

figure(6); clf;
plot(t, x_f(3, :) - l_0 * sin(x_f(5, :)));
hold on;
grid on;
plot(t, x_f(4, :));

figure(7); clf;
plot(t, y_f);
title('y_f');

% Animation:
style1 = hgexport('factorystyle');
style1.Format = 'png';
style1.Width = 6;
style1.Height = 2.5;
style1.Resolution = 300;
style1.Units = 'inch';
style1.FixedFontSize = 12;

c1 = [66, 80, 207] / 256;
c2 = [201, 40, 51] / 256;
c3 = [168, 56, 201] / 256;

fig = figure(8); clf;
set(gcf,'color','w');

for k = 1:250:K
    mass_pos = [x_f(1, k); x_f(3, k)];
    rod_end_pos = [x_f(1, k) - x_s(3, k) * cos(x_f(5, k)) x_f(3, k) - x_s(3, k) * sin(x_f(5, k))];
   
    plot([mass_pos(1) rod_end_pos(1)], [mass_pos(2) rod_end_pos(2)], 'LineStyle', '-', 'LineWidth', 3, 'Color', c3);
    grid on
    hold on
    plot(mass_pos(1), mass_pos(2), 'Marker', 'o', 'Color', c2, 'MarkerFaceColor', c2);
    plot(rod_end_pos(1), rod_end_pos(2), 'Marker', 'o', 'Color', c2, 'MarkerFaceColor', c2);
    axis equal
    axis([-0.5, 2.5, -0.5, 2.5]);
    
    drawnow
    hold off
end
