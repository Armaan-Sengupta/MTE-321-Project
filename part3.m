clear all;
clc;

function d = evaluate_equation(ma, tm, d_, Kf, Kfs, Se, Sut)
    % Equation parameters
    factor1 = (16 * 2) / pi;
    
    % First term inside the brackets
    term1 = (2 * Kf * d_ * ma) / Se;
    
    % Second term inside the brackets
    term2 = (sqrt(3) * Kfs * d_ * tm) / Sut;
    
    % Combine terms and evaluate the final expression
    d =  (factor1 *(term1 + term2))^(1/3);
end

diameter = 1.5   % Replace with actual value
ma = 8.8796   % Replace with actual value
tm = 3.6849   % Replace with actual value
kt = 2.1; % Rough Approximation: Need to change to be more accurate
kts = 1.625; % Rough Approximation: Need to change to be more accurate
notch_radius = 0.05*diameter;

Sut = 116
ka = 2.7*(Sut)^-0.265;
ke = 0.753;
kb = 0;
if (diameter < 2) kb = 0.879*(diameter^-0.107);
else kb = 0.910*(diameter^-0.157);
end
Se = ka*kb*ke*0.5*Sut

root_a_q = 0.2456 - 3.08*1e-3*Sut + 1.51*1e-5*Sut^2 - 2.67*1e-8*Sut^3;
root_a_qs = 0.19 - 2.51*1e-3*Sut + 1.35*1e-5*Sut^2 - 2.67*1e-8*Sut^3;


q = 1/(1 + root_a_q/sqrt(notch_radius));
qs = 1/(1 + root_a_qs/sqrt(notch_radius));    
kf = 1 + q*(kt - 1)
kfs = 1 + qs*(kts - 1)

fprintf("Shafter diameter = %.4f\n",evaluate_equation(ma, tm, diameter, kf, kfs, Se, Sut));
