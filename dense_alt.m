% dense_alt computes the density altitude based on static pressure and 
% temperature. This model is based on the standard atmosphere (troposphere 
% below 36089.24' and stratosphere above 36089.24').
% See Pitot Statics and the Standard Atmosphere by Erb Pg.21.
%
% H_rho = dense_alt(p, T, lam_p, b_p)
%
% INPUTS:
%   p_static: static pressure (lb/ft^2)
%   T: temperature (F)
%   lam_p: pressure sensor scale factor error (-)
%   b_p: pressure sensor bias error (lb/ft^2)
% 
% OUTPUTS:
%   H_rho: pressure altitude (ft)
%
% Sam Jaeger
% jaege246@umn.edu
% 8/5/2026

function H_rho = dense_alt(p, T, lam_p, b_p)
    %rho_SL =  0.002377; % slugs/ft^3
    p_SL =  2116.22; % lb/ft^2;
    T_SL = 288.15; % K

    T = (T + 459.67).*5/9; % F to K
    
    p_true = (p - b_p)./(1 + lam_p);
    delta = p_true./p_SL; % pressure ratio
    theta = T./T_SL; % temperature ratio
    sigma = delta./theta; % density ratio

    H_rho = (1 - sigma.^(1/4.2559))./(6.87559*10^(-6));

    i_strato = H_rho(H_rho > 36089.24); % find indices in the stratosphere
    if isempty(i_strato) == false
        H_rho(i_strato) = log(sigma(i_strato)./0.297075)./(-4.80637*10^(-5)) + 36089.24;
    end
end