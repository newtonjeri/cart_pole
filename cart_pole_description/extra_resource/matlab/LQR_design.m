 clc; 
 M = 2.7582989593438584919;  %% Mass of cart (kg)
 m = 0.96730709424135585817;  %% Mass of pendulum (kg)
 l = 0.5/2;  %% Length of pendulum (m)
 g = 9.81; %% Gravitational acceleration (m/s^2)

 %% x
 %% x_dot
 %% theta
 %% theta_dot

 A = [ 0  1  0  0;
       0  0  -((m*g)/(M))  0;
       0  0  0  1;
       0  0  (((M+m)*g)/(l*M))  0;
      ];

 B = [ 0;
       (1/M);
       0;
       (-(1)/(l*M));
      ];



Q = [ 2       0     0     0;
      0       2     0     0;
      0       0     200    0;
      0       0      0    2;
      ];

R = [1];

[K,P,E] = lqr(A,B,Q,R);

fprintf("%f, %f, %f, %f\n", K(1), K(2), K(3), K(4))

AA = A - B*K; % Closed-loop dynamics
BB = B;
CC = eye(4);  % Output all states
DD = zeros(4, 1); % No direct input to output

% Simulate with initial conditions
x0 = [0.1; 0; -0.1; 0]; % Initial conditions: small perturbations
t = 0:0.01:10;  % Simulation time

% Simulate the system using initial conditions
[y, t, x] = initial(ss(AA, B, CC, DD), x0, t);

% Plot the results
figure;
plot(t, x(:,1), t, x(:,2), t, x(:,3), t, x(:,4));
grid on;
title('Response Curves for x, x_d, \theta, \theta_d versus t');
xlabel('Time (sec)');
ylabel('State Variables');
legend('x (Cart Position)', 'x_d (Cart Velocity)', '\theta (Pendulum Angle)', '\theta_d (Pendulum Angular Velocity)');
