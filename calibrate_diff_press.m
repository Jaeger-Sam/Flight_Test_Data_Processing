% calibrate_diff_press.fcn finds the differential pressure scale factor
% and bias errors via ordinary least squares using GPS velocity and
% pitch/yaw attitude measurements. Additionally a constant wind vector in 
% the NED frame is needed. The measurement model assumes that...
%
%       Dp_meas = (1 + lam_Dp) * Dp_true + b_Dp
%
% Freestream pressure and temperature measurements are
% optional inputs and by default they are set to sea level conditions.
%
% [b_Dp, lam_Dp] = calibrate_diff_press(Dp_meas, V_NED, V_w_NED, theta, psi, plt_fit, p_inf, T_inf)
%
% INPUTS:
%   Dp_meas: Nx1 vector of differential pressure measured (lb/ft^2)
%   V_NED: Nx3 vector of inertial velocities in NED frame (ft/s)
%   V_w_NED: 3x1 vector of wind velocities in NED frame (ft/s)
%   theta: Nx1 vector of pitch attitude (deg)
%   psi: Nx1 vector of heading attitude (deg)
%
% OPTIONAL INPUTS:
%   plt_fit: true/false to plot fit
%   p_inf: static pressure measured, scalar or Nx1 vector (lb/ft^2) 
%   T_inf: Temperature measured, scalar or Nx1 vector (F)
% 
% OUTPUTS:
%   b_Dp: differential pressure bias error (lb/ft^2)
%   lam_Dp: static pressure scale factor error (-)
%
% Sam Jaeger
% jaege246@umn.edu
% 8/5/2026

function [ b_Dp, lam_Dp] = calibrate_diff_press(Dp_meas, V_NED, V_w_NED, theta, psi, varargin)
    
    % check dimensions of inputs
    sz_Dp = size(Dp_meas);

    if iscolumn(Dp_meas) == false
        error('Differential pressure must be a column vector')
    elseif sz_Dp(1) ~= length(theta)
        error('measured differential pressure and attitude do not match in length')
    end


    % sea level conditions
    p_SL = 2116.22; %lb/ft^2
    T_SL = 518.67; % R
    R = 1716.6; %specific gas constant for air

    narginchk(5,8)
    if nargin == 6
        plt_fit =  varargin{1};
        p_inf = p_SL;
        T_inf = T_SL;
    elseif nargin == 7
        plt_fit =  varargin{1};
        p_inf = varargin{2};
        T_inf = T_SL;
    elseif nargin == 8
        plt_fit =  varargin{1};
        p_inf = varargin{2};
        T_inf = varargin{3};
        T_inf = T_inf + 459.67; % measured temperature converted to Rankie
    else
        plt_fit =  false;
        p_inf = p_SL;
        T_inf = T_SL;
    end
    if iscolumn(p_inf) == false
        p_inf = p_inf';
    end
    if iscolumn(T_inf) == false
        T_inf = T_inf';
    end

    % only need u component, which isn't dependent on phi 
    uvw_GPS = GPS_SAD(V_NED, V_w_NED, zeros(length(theta),1), theta, psi);
    u_GPS = uvw_GPS(:,1); 

    Dp_GPS = p_inf.*(1 + (u_GPS.^2)./(7*R.*T_inf)).^(7/2) - p_inf;

    H = [Dp_GPS, ones(length(theta),1)];
    theta_hat = inv(H'*H)*H'*Dp_meas;
    
    lam_Dp = theta_hat(1) - 1;
    b_Dp = theta_hat(2);

    if plt_fit == true
        Dp_cal = (Dp_meas - b_Dp) / (1 + lam_Dp);
        figure(902)
        plot(Dp_meas,'*'); hold on
        plot(Dp_GPS,'o')
        plot(Dp_cal,'.')
        xlabel('timestep (k)','FontSize',15,'Interpreter','latex')
        ylabel('$\Delta p$ $(lb/ft^2)$','FontSize',15,'Interpreter','latex')
        legend('Measured','GPS "Truth"','Calibrated','Interpreter','latex')
        grid on
        hold off
    end
end