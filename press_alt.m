% press_alt computes the pressure altitude based on static pressure. This
% model is based on the standard atmosphere (troposphere below 36089.24' 
% and stratosphere above 36089.24').
% See Pitot Statics and the Standard Atmosphere by Erb Pg.20.
%
% H_c = press_alt(p, lam_p, b_p)
%
% INPUTS:
%   p_static: static pressure (lb/ft^2)
%   lam_p: pressure sensor scale factor error (-)
%   b_p: pressure sensor bias error (lb/ft^2)
% 
% OUTPUTS:
%   H_c: pressure altitude (ft)
%
% Sam Jaeger
% jaege246@umn.edu
% 12/11/2025
%   Revised: 4/8/2026
%   Revised: 8/5/2026 - Added stratosphere logic

function H_c = press_alt(p, lam_p, b_p)
    p_SL =  2116.22; % lb/ft^2;
    
    p_true = (p - b_p)./(1 + lam_p);
    delta = p_true/p_SL;

    H_c = (1 - delta.^(1/5.2559))./(6.87559*10^(-6));

    i_strato = H_c(H_c > 36089.24); % find indices in the stratosphere
    if isempty(i_strato) == false
        H_c(i_strato) = log(delta(i_strato)./0.223360)./(-4.80637*10^(-5)) + 36089.24;
    end
end