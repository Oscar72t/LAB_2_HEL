function T_D = mod_aero (theta_0 , theta_s , alpha_R, flight , geom)

%% Recall flight conditions parameters

rho = flight.rho;
Omega = flight.Omega;
V = flight.V;
W = flight.W;

%% Recall helicopter geometric parameters

n_b = geom.n_b;
c_0 = geom.c_0;
Cl_alpha = geom.Cl_alpha;
R = geom.R;

%% Compute parameters prior to thrust calculations

v_i = sqrt(W/(2*rho*geom.S)); % Induced velocity (estimated from hovering condition with MT)

mu_x = V/(Omega*R)*cos(alpha_R);

%% Calculate aerodynamic thrust 

x = linspace(0,1,1000); % Linspace to integrate
fun = v_i/(Omega*R).*x; % Define function to integrate using trapz

% fun_simp = v_i/(Omega*R);

T_D = n_b*1/4*rho*c_0*Cl_alpha*Omega^2*R^3*(mu_x*theta_s + theta_0*(2/3 + mu_x^2) + V/(Omega*R)*sin(alpha_R) - 2*trapz(x,fun));
% T_D = n_b*1/4*rho*c_0*Cl_alpha*Omega^2*R^3*(mu_x*theta_s + theta_0*(2/3 + mu_x^2) + V/(Omega*R)*sin(alpha_R) - fun_simp);

end