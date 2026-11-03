clc;
close all;

%% Extract Measured Input/Output Data

% Extract data from Simulink structures
time   = squeeze(t.signals.values);   % Time [s]
input  = squeeze(u.signals.values);   % Input voltage [V]
output = squeeze(y.signals.values);   % Motor speed [RPM]

% Force all data into column vectors
time   = time(:);
input  = input(:);
output = output(:);

% Ensure all vectors have the same length
N = min([length(time), length(input), length(output)]);

time   = time(1:N);
input  = input(1:N);
output = output(1:N);


%% Linear Least Squares to Estimate Discrete-Time Model Parameters

% First-order difference equation:
%
% y(k) = -a*y(k-1) + b*u(k-1)

% Construct regression matrix
Phi = [-output(1:end-1), input(1:end-1)];

% Dependent variable
Y = output(2:end);

% Estimate model parameters using Linear Least Squares
theta = (Phi' * Phi) \ (Phi' * Y);

% Extract estimated parameters
a = theta(1);
b = theta(2);

% Display estimated parameters
fprintf('Estimated parameter a = %.6f\n', a);
fprintf('Estimated parameter b = %.6f\n', b);


%% Simulate Identified Discrete-Time Model

y_sim = zeros(N,1);

% Use measured initial condition
y_sim(1) = output(1);

% Simulate model
for k = 2:N
    
    y_sim(k) = [-y_sim(k-1), input(k-1)] * theta;
    
end


%% Evaluate Model Fit

% Modelling error
error = output - y_sim;

% Mean modelling error
AverageError = mean(error);

% Mean absolute modelling error
MAE = mean(abs(error));


%% Graphical Outputs

figure

% ---------------------------------------------------------
% Input, measured output and simulated output
% ---------------------------------------------------------
subplot(2,1,1)

hh = plot(time, input, ...
          time, y_sim, ...
          time, output);

grid on
axis tight

xlabel('Time [s]', 'FontSize', 24)
ylabel('Signal', 'FontSize', 24)

legend('Input voltage, u [V]', ...
       'Simulated motor speed, y_{sim} [RPM]', ...
       'Measured motor speed, y [RPM]', ...
       'Location', 'best')

set(gca, 'FontSize', 20)

set(hh(1), 'LineWidth', 2, ...
           'Color', 'red', ...
           'LineStyle', '-')

set(hh(2), 'LineWidth', 2, ...
           'Color', 'blue', ...
           'LineStyle', '-')

set(hh(3), 'LineWidth', 2, ...
           'Color', 'black', ...
           'LineStyle', 'none', ...
           'Marker', '.')


% ---------------------------------------------------------
% Modelling error
% ---------------------------------------------------------
subplot(2,1,2)

plot(time, error, 'LineWidth', 2)

grid on
axis tight

xlabel('Time [s]', 'FontSize', 24)
ylabel('Error [RPM]', 'FontSize', 24)

title(['Mean Error = ', num2str(AverageError,3), ...
       ' RPM   |   MAE = ', num2str(MAE,3), ' RPM'], ...
       'FontSize', 22)

set(gca, 'FontSize', 20)