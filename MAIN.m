clc; clear all; close all;

%% HELICOPTERS LAB 2


%% VARIABLE DEFINITION FOR BELL 412 MODEL
%Define the flight struct:

flight.W=5390*9.8; %[N]
flight.rho=1.0879; %[kg/m^3] It was found in the manual that the forward flight height is 2000 ft
flight.Omega=324; %[rpm]
flight.V=130*0.514444 ;  %[m/s] Note that the data found at the manual was given in knots

%Define the geometry struct
geom.Cl_alpha=0.091*180/pi; %[1/degree]
geom.R=7; %[m]
geom.c_0=1.05*0.3048; %[m] Data in the manual was in inches
geom.m_blade=79; %[kg]
geom.deltaX=0.15; %[m]
geom.deltaZ=1.5;  %[m]

inert_y = geom.m_blade*geom.R^2/4 + geom.m_blade*geom.R^2/12;
geom.I_y=inert_y;

f = 2.787;

%% Apply bisection method

% Estimate initial guesses

theta_0m = -15;
theta_0p = 15;

% Enter Trim module

[alpha_R, alpha_d, T_Dtrim, beta_c, beta_s] = mod_trim(flight, geom, f);

% Enter Blade Dynamics module (preliminar calcs of initial guesses in the bisection method)

[theta_sm , theta_cm , beta_0m] = mod_BD (alpha_R , beta_c , beta_s, theta_0m , flight , geom); % Calculate Blade Dynamics for minimum estimated twist
[theta_sp , theta_cp , beta_0p] = mod_BD (alpha_R , beta_c , beta_s, theta_0p , flight , geom); % Calculate Blade Dynamics for maximum estimated twist

% Enter Aerodynamics module (preliminar calcs of initial guesses in the bisection method)

T_Daero_m = mod_aero (theta_0m , theta_sm , flight , geom);
T_Daero_p = mod_aero (theta_0p , theta_sp , flight , geom);

errm = T_Daero_m - T_Dtrim;
errp = T_Daero_p - T_Dtrim;

if errm*errp > 0 
    
    error('Change your initial guesses.')

end

% Iteration

for iter = 1:maxiter

    theta_0 = (theta_0m + theta_0p) / 2;
    
    [theta_s , theta_c , beta_0] = mod_BD (alpha_R , beta_c , beta_s, theta_0 , flight , geom); % Enter BD module in the iteration
    T_D = mod_aero (theta_0 , theta_s , flight , geom); % Enter Aero module in the iteration

    err = T_D - T_Dtrim;

    if (abs(err) < tol)

        break

    end

    if err > 0

        theta_0p = theta_0;
        errp = err;

    else

        theta_0m = theta_0;
        errm = err;

    end

end

if iter == maxiter

    error('No convergence!')

end
