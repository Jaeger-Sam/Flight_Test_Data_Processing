% press_altimeter computes the altitude based on static pressure, altimeter
% setting, and static pressure sensor calibration. This model is based on 
% the standard atmosphere (troposphere below 36089.24' and stratosphere 
% above 36089.24'). See Pitot Statics and the Standard Atmosphere by Erb 
% Pg.20.
%
% H = press_altimeter(p, altimeter, lam_p, b_p)
%
% INPUTS:
%   p_static: static pressure (lb/ft^2)
%   altimeter: local altimeter setting (in Hg)
%   lam_p: pressure sensor scale factor error (-)
%   b_p: pressure sensor bias error (lb/ft^2)
% 
% OUTPUTS:
%   H: altitude (ft)
%
% Sam Jaeger
% jaege246@umn.edu
% 8/5/2026

function H = press_altimeter(p, altimeter, lam_p, b_p)
    p_SL =  altimeter./0.014139032344453; % in Hg to lb/ft^2;
    
    p_true = (p - b_p)./(1 + lam_p);
    delta = p_true/p_SL;

    H = (1 - delta.^(1/5.2559))/(6.87559*10^(-6));
    
    i_strato = H(H > 36089.24); % find indices in the stratosphere
    if isempty(i_strato) == false
        H(i_strato) = log(delta(i_strato)./0.223360)./(-4.80637*10^(-5)) + 36089.24;
    end
    % if H > 36089.24
    %     H = log(delta./0.223360)./(-4.80637*10^(-5)) + 36089.24;
    %     %error('Pressure altitude in stratosphere or above!') 
    % end
end