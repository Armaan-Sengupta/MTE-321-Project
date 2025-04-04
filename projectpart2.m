
clc;
clear all;
global Az Ay Ft Fa Fr Lt Lr Pt Pr torque_o torque_c torque_d weight Sy Sut;

power = 126769;

scaling_factor_gearO = 0.25;
scaling_factor_gearC = 1;
scaling_factor_gearD = 0.75;

speed = 76.123;
r_0 = 5;  %gear 0 radius
r_d = 10; %gear 0 radius
r_c = 10; %gear 0 radius

d_0 = 1.5; %dia of shaft at gear o

weight = 35;
N_M_TO_LBF_IN = 8.851;
Sy = 96000;
Sut = 116000;

%gear 0
torque_o = ((scaling_factor_gearO*power)/speed)*N_M_TO_LBF_IN;
Ft = torque_o/r_0;
Fr = (7/14)*Ft;
Fa = (6/14)*Ft;

%gear D
torque_d = ((scaling_factor_gearD*power)/speed)*N_M_TO_LBF_IN;
Lt = torque_d/r_d;
Lr = Lt*tan(deg2rad(20));


%gear C
torque_c = ((scaling_factor_gearC*power)/speed)*N_M_TO_LBF_IN;
Pt = torque_c/r_c;
Pr = Pt*tan(deg2rad(20));

Az = -1*(Lt*4.5 + Pt*10.5 + Ft*18)/24;
Ay = -1*((Lr - weight)*-4.5 + (Pr + weight)*10.5 + (Fr - weight)*-18 + Fa*5)/24;

%New Code

function [sigma_x, tau_xy] = return_sigmax_tauxy(x_position, diameter)
    global Az Ay Ft Fa Fr Lt Lr Pt Pr torque_o torque_c torque_d weight;
    
    Moment_y = 0;
    Moment_z = 0;
    torque = 0;
   
    if x_position == 6.5
        %keyseat gear 0
       Moment_y = (Az*6);
       Moment_z = -Ay*6;
       torque = torque_o;
    elseif x_position == 7
       Moment_y = (Az*6 - 667.848*0.5);
       Moment_z = -Ay*6 + -5*Fa + 330.35*0.5;
       torque = torque_o;
    elseif x_position == 14     
       Moment_y = (Az*6 + (Az + Ft)*7.5);
       Moment_z = -1*Ay*6 - 5*Fa + 7.5*(-1*Ay - weight + Fr);
       torque = torque_o - torque_c;
    elseif x_position == 14.5
       Moment_y = (Az*6 + (Az + Ft)*7.5) + 0.5*806.86;
       Moment_z = -1*Ay*6 - 5*Fa + 7.5*(-1*Ay - weight + Fr) - 241.10416*0.5; 
       torque = torque_o - torque_c;
    elseif x_position == 19.5
       Moment_y = (Az*6 + (Az + Ft)*7.5 + (Az + Ft + Pt)*5.5);
       Moment_z = -1*Ay*6 - 5*Fa + 7.5*(-1*Ay - weight + Fr) + 5.5*(-1*Ay - weight + Fr - Pr - weight);
       torque = torque_o - torque_c;

    elseif x_position == 20
       Moment_y = (Az*6 + (Az + Ft)*7.5 + (Az + Ft + Pt)*6);
       Moment_z = -1*Ay*6 - 5*Fa + 7.5*(-1*Ay - weight + Fr) + 6*(-1*Ay - weight + Fr - Pr - weight);
       torque = torque_o - torque_c;
       
    end
    
    net_moment = sqrt(Moment_y^2 + Moment_z^2);
    sigma_x = abs(32*net_moment/(pi*diameter^3));
    tau_xy = abs(16*torque/(pi*diameter^3));
    
end


function [safety_factor] = return_safetyfactor_yield(sigma_x, tau_xy, kts, kt, notch_radius, Sy, Sut) 
    Sut = Sut/1000;
    root_a_q = 0.2456 - 3.08*1e-3*Sut + 1.51*1e-5*Sut^2 - 2.67*1e-8*Sut^3;
    root_a_qs = 0.19 - 2.51*1e-3*Sut + 1.35*1e-5*Sut^2 - 2.67*1e-8*Sut^3;
    Sut = Sut*1000;

    q = 1/(1 + root_a_q/sqrt(notch_radius));
    qs = 1/(1 + root_a_qs/sqrt(notch_radius));    
    kf = 1 + q*(kt - 1);
    kfs = 1 + qs*(kts - 1);
    von_mises_stress = sqrt((kf*sigma_x)^2 + 3*(kfs*tau_xy)^2);
    safety_factor = Sy / von_mises_stress;
end


function [safety_factor] = return_safetyfactor_goodman(sigma_x, tau_xy, kts, kt, notch_radius, Sy, Sut, diameter)
    Sut = Sut/1000;
    root_a_q = 0.2456 - 3.08*1e-3*Sut + 1.51*1e-5*Sut^2 - 2.67*1e-8*Sut^3;
    root_a_qs = 0.19 - 2.51*1e-3*Sut + 1.35*1e-5*Sut^2 - 2.67*1e-8*Sut^3;

    Sut = Sut*1000;
    
    q = 1/(1 + root_a_q/sqrt(notch_radius));
    qs = 1/(1 + root_a_qs/sqrt(notch_radius));    
    kf = 1 + q*(kt - 1);
    kfs = 1 + qs*(kts - 1);

    ka = 2.7*(Sut/1000)^-0.265;
    ke = 0.753;
    kb = 0;
    if (diameter < 2) kb = 0.879*(diameter^-0.107);
    else kb = 0.910*(diameter^-0.157);
    end

    Se = ka*kb*ke*0.5*Sut
    SigmaA_ = kf*sigma_x
    SigmaM_ = kfs*tau_xy*sqrt(3)
    safety_factor = 1 / (SigmaM_ / Sut + SigmaA_ / Se);
end

global x_position kt kts notch_radius diameter

function dispSafety(label)
    global x_position kt kts notch_radius diameter Sy Sut;
    
    fprintf('%s\n', label);
    [sigma_x, tau_xy] = return_sigmax_tauxy(x_position,diameter)
    sigma_x = abs(sigma_x);
    tau_xy = abs(tau_xy);
    safety_factor_yield = return_safetyfactor_yield(sigma_x, tau_xy, kts, kt, notch_radius, Sy, Sut);
    safety_factor_goodman = return_safetyfactor_goodman(sigma_x, tau_xy, kts, kt, notch_radius, Sy, Sut,diameter)
    fprintf("Safety Factor Yield: %.4f\n",safety_factor_yield);
    fprintf("Safety Factor Fatigue: %.4f\n",safety_factor_goodman);
    fprintf('-----------------\n');
end

x_position = 14;
kt = 2.14;
kts = 3.0;
notch_radius = 0.0375;
diameter = 1.875;
dispSafety("Gear C Keyseat");


x_position = 6.5;
kt = 2.14;
kts = 3.0;
notch_radius = 0.03;
diameter = 1.5;
dispSafety("Gear O Keyseat");


x_position = 20;
kt = 2.14;
kts = 3.0;
notch_radius = 0.0375;
diameter = 1.875;
dispSafety("Gear D Keyseat");


x_position = 14;
kt = 2.1;  
kts = 1.6;
notch_radius = 0.09375;
diameter = 1.875;
dispSafety("Gear C Shoulder Fillet");

x_position = 7;
kt = 2.1; % Rough Approximation: Need to change to be more accurate
kts = 1.625; % Rough Approximation: Need to change to be more accurate
notch_radius = 0.075;
diameter = 1.5;
dispSafety("Gear O Shoulder Fillet");


x_position = 19.5;
kt = 2.05;  % Rough Approximation: Need to change to be more accurate
kts = 1.6; % Rough Approximation: Need to change to be more accurate
notch_radius = 0.09375;
diameter = 1.875;
dispSafety("Gear D Shoulder Fillet");


% Gear O keyseat
%Moment_y = Az*6;
%Moment_z = Ay*6;

% Gear C keyseat


%net_moment = sqrt(Moment_y^2 + Moment_z^2);
%sigma_x = 32*net_moment/(pi*d_0^3);
%tau = 16*torque_o/(pi*d_0^3);
%sigma_von_mises = sqrt(sigma_x^2 + 3*tau^2);

%eq = sigma_von_mises == Sy;
%disp(vpa(solve(eq,power)));

%n = Sy/sigma_von_mises

