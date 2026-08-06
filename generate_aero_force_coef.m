% generate_aero_force_coef.fcn calculates the 
%
% INPUTS:
%   a_x: Nx1 vector of x accelerations from the IMU (ft/s^2)
%   a_y: Nx1 vector of y accelerations from the IMU (ft/s^2)
%   a_z: Nx1 vector of z accelerations from the IMU (ft/s^2)
%   u_inf: Nx1 vector of freestream airspeed (ft/s)
%   alpha: Nx1 vector of angle of attacks (rad)
%   beta: Nx1 vector of angle of sideslips (rad)
%   T: Nx1 vector of thrust (lb)
%   rho_inf: freestream air density (slugs/ft^3)
%   W: weight of aircraft (lb)
%   Sw: wing area (ft^2)
% 
% OPTIONAL INPUTS:
%   b_a_xyz: 3x1 vector of accelerometer biases (ft/s^2)
%   plt_thist: true/false to plot time history of coefficients
%
% OUTPUTS:
%   C_s: Nx3 matrix of stability coefficients with columns [C_D C_Y_W  C_L]
%   C_b: Nx3 matrix of body fixed coefficients with columns [C_X  C_Y  C_Z]
%
% Sam Jaeger
% jaege246@umn.edu
% 8/5/2026

function [C_s, C_b] = generate_aero_force_coef(ax, ay, az, u_inf, alpha, beta, T, rho_inf, W, Sw, varargin)
    N = length(ax);
    
    narginchk(10,12)
    if nargin == 11
        b_a_xyz = varargin{1};
        plt_thist = false;
    elseif nargin == 12
        b_a_xyz = varargin{1};
        plt_thist =varargin{2};
    else
        b_a_xyz = zeros(3,1);
        plt_thist = false;
    end
    g = 32.174;
    m = W/g;
    
    C_b = zeros(N,3);
    C_s = zeros(N,3);
    for ii=1:N
        nondim = 2*m*(( cos(beta(ii))*cos(alpha(ii))).^2)./rho_inf./(u_inf(ii).^2)./Sw;
        C_b_i = [ax(ii) - b_a_xyz(1) - T(ii)/m; 
            ay(ii) - b_a_xyz(2); 
            az(ii) - b_a_xyz(3)]*nondim;
        C_b(ii,:) = C_b_i';

        C_s_i = coef_body_to_stab(C_b_i',alpha(ii),beta(ii));
        C_s(ii,:) = C_s_i';
    end

    if plt_thist == true
        figure(910)
        subplot(3,1,1)
        plot(C_s(:,1),'.'); grid on;
        xlabel('timestep','FontSize',15,'Interpreter','latex')
        ylabel('$C_D$','FontSize',15,'Interpreter','latex')
        subplot(3,1,2)
        plot(C_s(:,2),'.'); grid on;
        xlabel('timestep','FontSize',15,'Interpreter','latex')
        ylabel('$C_{Y_w}$','FontSize',15,'Interpreter','latex')
        subplot(3,1,3)
        plot(C_s(:,3),'.'); grid on;
        xlabel('timestep','FontSize',15,'Interpreter','latex')
        ylabel('$C_{L}$','FontSize',15,'Interpreter','latex')
    end
end