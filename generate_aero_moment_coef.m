% generate_aero_moment_coef generates the moment coefficients given time
% histories of body fixed angular rates, air data, and aircraft geometry
% parameters. This function will numerically differentiate p,q,r after
% applying a low pass filter (see ndiff_signal).
%
% [C_lmn]= generate_aero_moment_coef(p,q,r, u_inf,alpha,beta, rho_inf, Ixx,Iyy,Izz,Ixz, Sw,cbw,bw, b_g_xyz, plt_thist, M_prop, method, f_max)
%
% INPUTS:
%   p: Nx1 vector of x ang velocity from the IMU (deg/s)
%   q: Nx1 vector of y ang velocity from the IMU (deg/s)
%   r: Nx1 vector of z ang velocity from the IMU (deg/s)
%   u_inf: Nx1 vector of freestream airspeed (ft/s)
%   alpha: Nx1 vector of angle of attacks (deg)
%   beta: Nx1 vector of angle of sideslips (deg)
%   rho_inf: freestream air density (slugs/ft^3)
%   Ixx: moment of inertia about x axis (slugs-ft^2)
%   Iyy: moment of inertia about y axis (slugs-ft^2)
%   Izz: moment of inertia about z axis (slugs-ft^2)
%   Ixz: product of inertia about x & z axis (slugs-ft^2)
%   Sw: wing area (ft^2)
%   cbw: mean aerodynamic chord (ft)
%   bw: wingspan (ft)
%
% OPTIONAL INPUTS:
%   b_g_xyz: 3x1 vector of gyro biases (deg/s)
%   plt_thist: true/false to plot time history of coefficients
%   M_prop: Nx3 matrix of propeller moments, default is zero (ft*lb)
%   method: numerical differentiation method (input to ndiff_signal.fcn)
%   f_max: cutoff frequency for low pass filter (input to ndiff_signal.fcn)
%
% OUTPUTS
%   C_lmn: Nx3 matrix of body fixed moment coefficients 
%
% Sam Jaeger
% jaege246@umn.edu
% 8/10/2026

function [C_lmn]= generate_aero_moment_coef(p,q,r, u_inf,alpha,beta, rho_inf, Ixx,Iyy,Izz,Ixz, Sw,cbw,bw, varargin)
    N = length(p);
    
    narginchk(14,19)
    if nargin == 15
        b_g_xyz = varargin{1};
        plt_thist = false;
        M_prop = zeros(N,3);
        method = 1;
        f_max = [];
    elseif nargin == 16
        b_g_xyz = varargin{1};
        plt_thist =varargin{2};
        M_prop = zeros(N,3);
        method = 1;
        f_max = [];
    elseif nargin == 17
        b_g_xyz = varargin{1};
        plt_thist =varargin{2};
        M_prop =varargin{3};
        method = 1;
        f_max = [];
    elseif nargin == 18
        b_g_xyz = varargin{1};
        plt_thist =varargin{2};
        M_prop =varargin{3};
        method = varargin{4};
        f_max = [];
    elseif nargin == 19
        b_g_xyz = varargin{1};
        plt_thist =varargin{2};
        M_prop =varargin{3};
        method = varargin{4};
        f_max = varargin{5};
    else
        b_g_xyz = zeros(3,1);
        plt_thist = false;
        M_prop = zeros(N,3);
        method = 1;
        f_max = [];
    end
    if isempty(M_prop)
        M_prop = zeros(N,3);
    end

    % convert to rad & Remove bias
    alpha = alpha*pi/180; 
    beta = beta*pi/180;
    p = (p - b_g_xyz(1))*pi/180;
    q = (q - b_g_xyz(2))*pi/180;
    r = (r - b_g_xyz(3))*pi/180;
    
    % Numerically differentiate pqr
    dt = 0.01; % fix at 100 Hz for n580 & FMUR
    pdot = ndiff_signal(p,method,dt,f_max);
    qdot = ndiff_signal(q,method,dt,f_max);
    rdot = ndiff_signal(r,method,dt,f_max);

    % inertia matrix
    I_mat = [Ixx, 0, -Ixz;
             0,  Iyy, 0
             -Ixz, 0, Izz];
    
    C_lmn = zeros(N,3); 
    for ii=1:N
        if ii==1 || ii==N
            omega_dot = [0; 0; 0];
        else
            omega_dot = [pdot(ii-1); qdot(ii-1); rdot(ii-1)];
        end

        % Moments
        M = I_mat*omega_dot - [(Iyy-Izz)*q(ii)*r(ii) + Ixz*p(ii)*q(ii);  (Izz-Ixx)*p(ii)*r(ii) + Ixz*((r(ii)^2) - (p(ii)^2)); (Ixx-Iyy)*p(ii)*q(ii) - Ixz*p(ii)*r(ii)];
        
        % Nondimensionalize
        qbar = 0.5*rho_inf.*(u_inf(ii)./cos(alpha(ii))./cos(beta(ii))).^2;
        C_lmn(ii,1) = (M(1) - M_prop(ii,1))/(qbar*Sw*bw);
        C_lmn(ii,2) = (M(2) - M_prop(ii,2))/(qbar*Sw*cbw);
        C_lmn(ii,3) = (M(3) - M_prop(ii,3))/(qbar*Sw*bw);
    end
    

    % plot response
    if plt_thist == true
        figure(960)
        h(1)=subplot(3,1,1);
        plot(C_lmn(:,1),'.'); grid on;
        xlabel('timestep','FontSize',15,'Interpreter','latex')
        ylabel('$C_l$','FontSize',15,'Interpreter','latex')
        h(2)=subplot(3,1,2);
        plot(C_lmn(:,2),'.'); grid on;
        xlabel('timestep','FontSize',15,'Interpreter','latex')
        ylabel('$C_{m}$','FontSize',15,'Interpreter','latex')
        h(3)=subplot(3,1,3);
        plot(C_lmn(:,3),'.'); grid on;
        xlabel('timestep','FontSize',15,'Interpreter','latex')
        ylabel('$C_{n}$','FontSize',15,'Interpreter','latex')
        linkaxes(h,'x');
    end
end