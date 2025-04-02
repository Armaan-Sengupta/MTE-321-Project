%% Steps To Solve For Power

%% 1. Express von mises stress as function of power and reaction forces
%%    of bearing A

%% 2. Get One Big Massive Equation and then chuck everything in there bruh

%% Matlab equations
%syms power;

clc;
clear all;

power = 447244;

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


%gear o
Moment_y = Az*6;
Moment_z = Ay*6;

net_moment = sqrt(Moment_y^2 + Moment_z^2);
sigma_x = 32*net_moment/(pi*d_0^3);
tau = 16*torque_o/(pi*d_0^3);
sigma_von_mises = sqrt(sigma_x^2 + 3*tau^2);

%eq = sigma_von_mises == Sy;
%disp(vpa(solve(eq,power)));

n = Sy/sigma_von_mises

