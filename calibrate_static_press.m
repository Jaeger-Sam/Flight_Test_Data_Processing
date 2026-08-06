% calibrate_static_press.fcn computes the static pressure sensor bias and
% scale factor error given an INS altitude time history, and current
% altimeter setting. Default will be set the scale factor error to zero.
% The measurement model assumes:
%
%       p_static_meas = (1 + lam_p_static)*p_inf_true + b_p_static
%
% This function estimates the true freestream pressure (p_inf) based on the
% INS altitude, current altimeter setting, and a model of the standard
% atmosphere via least squares. The model for pressure altitude (corrected
% for nonstandard pressure) can be found in "Pitot-Statics and the Standard
% Atmosphere" by Erb on Pg. 20.
%
% [b_p_static, lam_p_static] = calibrate_static_press(p_static_meas, h_INS, altimeter, plt_fit, est_scale_factor)
%
% INPUTS:
%   p_static_meas: Nx1 vector of static pressure measured (lb/ft^2)
%   h_INS: Nx1 vector of geometric altitude from GPS/INS system (ft)
%   altimeter: local altimeter setting (in Hg)
%
% OPTIONAL INPUTS:
%   plt_fit: true/false to plot fit
%   est_scale_factor: true/false to estimate static pressure scale factor
% 
% OUTPUTS:
%   b_p_static: static pressure bias error (lb/ft^2)
%   lam_p_static: static pressure scale factor error (-)
%
% Sam Jaeger
% jaege246@umn.edu
% 8/5/2026


function [b_p_static, lam_p_static] = calibrate_static_press(p_static_meas, h_INS, altimeter, varargin)
    p_SL =  altimeter./0.014139032344453; % in Hg to lb/ft^2;

    % check sizes of inputs
    sz_p_static = size(p_static_meas);
    sz_h_INS = size(h_INS);
    if sz_p_static(1) ~= sz_h_INS(1)
        error('measured static pressure and INS altitudes do not match in length')
    elseif sz_p_static(2) ~= 1 || sz_h_INS(2) ~= 1
        error('measured static pressure and INS altitude must be column vectors')
    end

    % input logic
    narginchk(3,5)
    if nargin == 4
        plt_fit = varargin{1};
        est_scale_factor = false;
    elseif nargin == 5
        plt_fit = varargin{1};
        est_scale_factor = varargin{2};
    else
        plt_fit = false;
        est_scale_factor = false;
    end

    % calculate "true" freestream pressure estimated from GPS
    kT = 6.87559*10^(-6);
    kT1 = 5.2559;
    p_inf = p_SL*(1 - kT*h_INS).^(kT1);

    if est_scale_factor == true
        H = [p_inf, ones(length(p_inf),1)]; % measurement matrix
        theta = inv(H'*H)*H'*p_static_meas;
        lam_p_static = theta(1) - 1;
        b_p_static = theta(2);
    else
        b_p_static = mean(p_static_meas-p_inf);
        lam_p_static = 0;
    end

    if plt_fit == true
        p_static_cal = (p_static_meas - b_p_static) / (1 + lam_p_static);
        figure(901)
        plot(p_static_meas,'*'); hold on
        plot(p_inf,'o')
        plot(p_static_cal,'.')
        xlabel('timestep (k)','FontSize',15,'Interpreter','latex')
        ylabel('$p_{static}$ $(lb/ft^2)$','FontSize',15,'Interpreter','latex')
        legend('Measured','GPS "Truth"','Calibrated','Interpreter','latex')
        grid on
        hold off
    end
end