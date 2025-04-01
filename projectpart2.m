%% Steps To Solve For Power

%% 1. Express von mises stress as function of power and reaction forces
%%    of bearing A

%% 2. Get One Big Massive Equation and then chuck everything in there bruh

%% Matlab equations
syms power;

scaling_factor_gearO = 0.25;
scaling_factor_gearC = 1;
scaling_factor_gearD = 0.75;
chosen_scaling_factor = scaling_factor_gearO;
speed = 726.92;
diameter = 10;
radius = 5;
weight = 35;

Lt = scaling_factor_gearD*power/(speed*radius);
Pt = scaling_factor_gearC*power/(speed*radius);
Ft = scaling_factor_gearO*power/(speed*radius);
Lr = Lt*tan(0.349066);
Pr = Pt*tan(0.349066);
Fr = 0.5*Ft;
Fa = 2.33333*Ft;
Az = -1*(Lt*4.5 + Pt*10.5 + Ft*18)/24;
Ay = -1*(weight*4.5 - Lr*4.5 + Pr*10.5 + weight*10.5 + weight*18 - Fr*18 - Fa*5)/24;

Moment_y = Ft*6 + Az*0.5;
Moment_z = Fr*6 + Fa*5 - Ay*0.5;

Moment = sqrt(Moment_y^2 + Moment_z^2);
Torque = chosen_scaling_factor*power/speed;
sigma_x = 32*Moment/(pi*diameter^3);
tau = 16*Torque/(pi*diameter^3);

eq = sqrt(sigma_x^2 + 3*tau^2) == 9600;

sol = solve(eq, power);
disp(sol);


