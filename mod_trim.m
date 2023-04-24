
%{
    DESCRIPTION: 
        -> Trim module
    
    ASSUMPTIONS
        ->No tail rotor effect
        -> T=W
        -> Small angle approximation

    INPUTS: 
        -> Flight condition struct retrieving every variable associated to the helicopter flight [-] (Struct)
        -> Geometry struct retrieving every dimension needed [-] (struct)
        -> f: Airframe equivalent flat plate area  [-] (double)
    
    OUTPUTS: 
        -> Alpha_d: Tilting of the rotor disk w.r.t. horizon:  [-] (double)
        -> Thrust at the rotor disk [-] (double)
       
%}

function [alpha_r, alpha_d, T_d, beta_c, beta_s] = mod_trim(flight, geom, f)


%% TRIM EQUATIONS DEFINED AT U13
T_d=flight.W;
Df=0.5 * flight.rho * flight.V^2 * f;
alpha_d= -Df/flight.W;


beta_c=atan(geom.deltaX/geom.deltaZ);
beta_s=0; %NOTE THAT THE TAIL IS ASSUMED TO BE 0
%This part of the module is left if the difficulty of the effect of the
%tail rotor on trim is considered.

alpha_r=beta_c+ alpha_d;


