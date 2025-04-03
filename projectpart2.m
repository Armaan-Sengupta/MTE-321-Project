%% Steps To Solve For Power

%% 1. Express von mises stress as function of power and reaction forces
%%    of bearing A

%% 2. Get One Big Massive Equation and then chuck everything in there bruh

%% Matlab equations

%syms power;

clc;
clear all;

power = 170;

scaling_factor_gearO = 0.25;
scaling_factor_gearC = 1;
scaling_factor_gearD = 0.75;


speed = 76.123;
r_0 = 5;  %gear 0 radius
r_d = 10; %gear 0 radius
r_c = 10; %gear 0 radius

d_0 = 1.5; %dia of shaft at gear o

weight = 35;
N_M_TO_LBF_IN = 550;
Sy = 96000;
Sut = 116;

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
disp(torque_c);
Pt = torque_c/r_c;
Pr = Pt*tan(deg2rad(20));

Az = -1*(Lt*4.5 + Pt*10.5 + Ft*18)/24;
Ay = -1*((Lr - weight)*-4.5 + (Pr + weight)*10.5 + (Fr - weight)*-18 + Fa*5)/24;

%New Code

function [sigma_x, tau_xy] = return_sigmax_tauxy(x_position, torque, Az, Ay, Fa, Ft, Fr, Pt, Pr, weight, diameter)

    Az = 1404.8;
    Ft = 736.96;
    Ay = -3.154;
    Fr = 368.48;
    Fa = 315.84;

    if x_position == 6
       Moment_y = Az*6;
       Moment_z = Ay*6;
    elseif x_position == 6.5
       Moment_y = Az*6 - 667.848*0.5;
       Moment_z = -5*Fa + 330.35*0.5;
    elseif x_position == 13.5     
       Moment_y = Az*6 + (Az - Ft)*7.5;
       Moment_z = -5*Fa + Ay*6 + 7.5*(Ay - weight + Fr);
       
    elseif x_position == 14
       Moment_y = Az*6 + (Az - Ft)*7.5 - 0.5*806.86;
       Moment_z = -5*Fa + 7.5*(Ay - weight + Fr) - 241.10416*0.5; % NEED TO CHANGE;
    elseif x_position == 19.5
       Moment_y = Az*6 + (Az - Ft)*7.5 - (Az - Ft - Pt)*6;
       Moment_z = -5*Fa + 7.5*(Ay - weight + Fr) - 6*(Ay - weight + Fr + Pr); % NEED TO CHANGE;
    elseif x_position == 20
       Moment_y = Az*6 + (Az - Ft)*7.5 - (Az - Ft - Pt)*6 - 1910.79*0.5;
       Moment_z = -5*Fa + 7.5*(Ay - weight + Fr) - 6*(Ay - weight + Fr + Pr) + 0.5*125.7; 
    end
    
    net_moment = sqrt(Moment_y^2 + Moment_z^2);
    sigma_x = 32*net_moment/(pi*diameter^3);
    tau_xy = 16*torque/(pi*diameter^3);
    
end


function [safety_factor] = return_safetyfactor(sigma_x, tau_xy, kts, kt, notch_radius, Sy, Sut)    
    root_a_q = 0.2456 - 3.08*1e-3*Sut + 1.51*1e-5*Sut^2 - 2.67*1e-8*Sut^3;
    root_a_qs = 0.19 - 2.51*1e-3*Sut + 1.35*1e-5*Sut^2 - 2.67*1e-8*Sut^3;
    
    q = 1/(1 + root_a_q/sqrt(notch_radius));
    qs = 1/(1 + root_a_qs/sqrt(notch_radius));    
    kf = 1 + q*(kt - 1);
    kfs = 1 + qs*(kts - 1);
    von_mises_stress = sqrt((kf*sigma_x)^2 + 3*(kfs*tau_xy)^2);
    safety_factor = Sy / von_mises_stress;
end

%Gear C keyseat

x_position = 13.5;
kt = 2.14;
kts = 3.0;
notch_radius = 0.0375;
torque = torque_c;
diameter = 1.875
[sigma_x_gearc_keyseat, tau_xy_gearc_keyseat] = return_sigmax_tauxy(x_position, torque, Az, Ay, Fa, Ft, Fr, Pt, Pr, weight, diameter);
[safety_factor] = return_safetyfactor(sigma_x_gearc_keyseat, tau_xy_gearc_keyseat, kts, kt, notch_radius, Sy, Sut);

%Gear O keyseat

x_position = 6;
kt = 2.14;
kts = 3.0;
notch_radius = 0.03;
torque = torque_o;
diameter = 1.5;
[sigma_x_gearc_keyseat, tau_xy_gearc_keyseat] = return_sigmax_tauxy(x_position, torque, Az, Ay, Fa, Ft, Fr, Pt, Pr, weight, diameter);
[safety_factor] = return_safetyfactor(sigma_x_gearc_keyseat, tau_xy_gearc_keyseat, kts, kt, notch_radius, Sy, Sut);

%Gear D keyseat

x_position = 19.5;
kt = 2.14;
kts = 3.0;
notch_radius = 0.0375;
torque = torque_d;
diameter = 1.875;
[sigma_x_gearc_keyseat, tau_xy_gearc_keyseat] = return_sigmax_tauxy(x_position, torque, Az, Ay, Fa, Ft, Fr, Pt, Pr, weight, diameter);
[safety_factor] = return_safetyfactor(sigma_x_gearc_keyseat, tau_xy_gearc_keyseat, kts, kt, notch_radius, Sy, Sut);

%Gear C shoulder fillet

x_position = 14;
kt = 1.9;  % Rough Approximation: Need to change to be more accurate
kts = 3.0; % Rough Approximation: Need to change to be more accurate
notch_radius = 0.0375;
torque = torque_c;
diameter = 1.875;
[sigma_x_gearc_shoulderfillet, tau_xy_gearc_shoulderfillet] = return_sigmax_tauxy(x_position, torque, Az, Ay, Fa, Ft, Fr, Pt, Pr, weight, diameter);
[safety_factor] = return_safetyfactor(sigma_x_gearc_shoulderfillet, tau_xy_gearc_shoulderfillet, kts, kt, notch_radius, Sy, Sut);

%Gear O shoulder fillet

x_position = 6.5;
kt = 2.1; % Rough Approximation: Need to change to be more accurate
kts = 3.0; % Rough Approximation: Need to change to be more accurate
notch_radius = 0.03;
torque = torque_o;
diameter = 1.5;
[sigma_x_gearo_shoulderfillet, tau_xy_gearo_shoulderfillet] = return_sigmax_tauxy(x_position, torque, Az, Ay, Fa, Ft, Fr, Pt, Pr, weight, diameter);
[safety_factor] = return_safetyfactor(sigma_x_gearo_shoulderfillet, tau_xy_gearo_shoulderfillet, kts, kt, notch_radius, Sy, Sut);

%Gear D shoulder fillet

x_position = 20;
kt = 1.9; % Rough Approximation: Need to change to be more accurate
kts = 3.0; % Rough Approximation: Need to change to be more accurate
notch_radius = 0.0375;
torque = torque_d;
diameter = 1.875;
[sigma_x_geard_keyseat, tau_xy_geard_keyseat] = return_sigmax_tauxy(x_position, torque, Az, Ay, Fa, Ft, Fr, Pt, Pr, weight, diameter);
[safety_factor] = return_safetyfactor(sigma_x_geard_keyseat, tau_xy_geard_keyseat, kts, kt, notch_radius, Sy, Sut);


disp(safety_factor);

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

