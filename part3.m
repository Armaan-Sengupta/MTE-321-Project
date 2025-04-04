clear all;
clc;

% Define the function to evaluate the diameter
function d = evaluate_equation(ma, tm, Kf, Kfs, Se, Sut)
    % Equation parameters
    factor1 = (16 * 2) / pi;
    
    % First term inside the brackets
    term1 = (2 * Kf * ma) / Se;
    
    % Second term inside the brackets
    term2 = (sqrt(3) * Kfs * tm) / Sut;
    
    % Combine terms and evaluate the final expression
    d = (factor1 * (term1 + term2))^(1/3);
end

% Initial parameters
diameter = 1.5;  % Initial guess
ma = 8.8796;
tm = 3.6849;
kt = 2.1;
kts = 1.625;
Sut = 116;
convergence_threshold = 1e-6;
max_iterations = 100;

% Calculate Se
ka = 2.7 * (Sut)^-0.265;
ke = 0.753;
kb = 0;
if (diameter < 2)
    kb = 0.879 * (diameter^-0.107);
else
    kb = 0.910 * (diameter^-0.157);
end
Se = ka * kb * ke * 0.5 * Sut;

% Pre-allocate for speed
diameters = zeros(1, max_iterations);
iterations = 1:max_iterations;

% Iterative diameter calculation
for i = 1:max_iterations
    notch_radius = 0.05 * diameter;
    root_a_q = 0.2456 - 3.08e-3 * Sut + 1.51e-5 * Sut^2 - 2.67e-8 * Sut^3;
    root_a_qs = 0.19 - 2.51e-3 * Sut + 1.35e-5 * Sut^2 - 2.67e-8 * Sut^3;
    q = 1 / (1 + root_a_q / sqrt(notch_radius));
    qs = 1 / (1 + root_a_qs / sqrt(notch_radius));
    kf = 1 + q * (kt - 1);
    kfs = 1 + qs * (kts - 1);
    
    % Calculate new diameter
    new_diameter = evaluate_equation(ma, tm, kf, kfs, Se, Sut);
    diameters(i) = new_diameter;
    
    % Check for convergence
    if abs(new_diameter - diameter) < convergence_threshold
        fprintf('Converged at iteration %d: Diameter = %.4f\n', i, new_diameter);
        diameters = diameters(1:i); % Trim the unused part
        iterations = iterations(1:i); % Trim the iteration numbers
        break;
    end
    % Update diameter for next iteration
    diameter = new_diameter;
end

% Plotting the diameter vs. iteration number
figure;
plot(iterations, diameters, '-o');
xlabel('Iteration Number');
ylabel('Diameter');
title('Convergence of Diameter');
grid on;