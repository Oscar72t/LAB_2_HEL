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
