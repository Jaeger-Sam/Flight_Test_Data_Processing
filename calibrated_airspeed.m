% calibrated_airspeed computes calibrated airspeed in US units
% given a differential pressure for subsonic conditions. Calibrated airspeed 
% is the airspeed based on true differential pressure measurements with
% respect to sea level conditions. See Eqn C65 in "Pitot-Statics and the
% Standard Atmosphere" by Erb.
%
% INPUTS:
%   Dp: Difference between total and ambient pressure in lb/ft^2
%       Dp=(p_T - p_a)
%   lam_Dp: pressure sensor scale factor error
%   b_Dp: pressure sensor bias error
%
% OUTPUTS:
%   V_c: calibrated airspeed in ft/s
%
% Sam Jaeger
% jaege246@umn.edu
% 4/8/2026

function V_c = calibrated_airspeed(Dp,lam_Dp,b_Dp)
    % Table 2.1 in Erb
    rho_SL = 0.00237688; % slugs / ft^3
    p_SL = 2116.22; % lb/ft^2

    Dp_true = (Dp - b_Dp)./(1 + lam_Dp);
    V_c = sqrt((7.*p_SL./rho_SL).*( ((Dp_true./p_SL)+1 ).^(2/7) - 1) );
end