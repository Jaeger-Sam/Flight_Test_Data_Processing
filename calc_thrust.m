% calc_thrust.fcn calculates thrust of a fixed-pitch propeller driven
% aircraft given rpm, air data, and propeller information.
%
% [T,CT,J] = calc_thrust(rpm,u_inf,alpha,beta,rho_inf,p_CT,d,plt_thrust)
%
% INPUTS:
%   rpm: Nx1 vector of propeller rpm (rev/min)
%   u_inf: Nx1 vector of airspeed in the body fixed x direction (ft/s)
%   alpha: Nx1 vector of angle of attack (deg)
%   beta: Nx1 vector of sideslips (deg)
%   rho_inf: scalar air density (slugs/ft^3)
%   p_CT: polynomial vector of thrust coefficient used to evaluate CT
%   d: propeller diameter (ft)
%
% OPTIONAL INPUTS:
%   plt_thrust: true/false to plot time history of thrust
%
% OUTPUTS:
%   T: Nx1 vector of thrust (lb)
%   CT: Nx1 vector of thrust coefficients (-)
%   J: Nx1 vector of advance ratio (-)
%
% Sam Jaeger
% jaege246@umn.edu
% 8/6/2026

function [T,CT,J] = calc_thrust(rpm,u_inf,alpha,beta,rho_inf,p_CT,d,varargin)
    
    narginchk(7,8)
    if nargin == 8 
        plt_thrust = varargin{1};
    else
        plt_thrust = false;
    end

    % check inputs for column vectors
    if iscolumn(rpm) == false
        rpm = rpm';
    end
    if iscolumn(u_inf) == false
        u_inf = u_inf';
    end
    if iscolumn(alpha) == false
        alpha = alpha';
    end
    if iscolumn(beta) == false
        beta = beta';
    end

    % calculations
    V_inf = u_inf./cosd(alpha)./cosd(beta); % total airspeed
    n = rpm/60; % rev/s
    J = V_inf./n./d; % advance ratio

    CT = polyval(p_CT,J);
    T = rho_inf.*(n.^2).*(d.^4).*CT;
    if plt_thrust == true
        figure(920); hold on
        h(1)=subplot(3,1,1);
        plot(T,'.'); grid on;
        xlabel('timestep','FontSize',15,'Interpreter','latex')
        ylabel('$T$ $(lb)$','FontSize',15,'Interpreter','latex')
        h(2)=subplot(3,1,2);
        plot(CT,'.'); grid on;
        xlabel('timestep','FontSize',15,'Interpreter','latex')
        ylabel('$C_T$','FontSize',15,'Interpreter','latex')
        h(3)=subplot(3,1,3);
        plot(J,'.'); grid on;
        xlabel('timestep','FontSize',15,'Interpreter','latex')
        ylabel('$J$','FontSize',15,'Interpreter','latex')
        linkaxes(h,'x');
    end
end