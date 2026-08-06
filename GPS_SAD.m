% GPS_SAD.fcn computes the body fixed uvw air velocities given a
% North-East-Down velocity solution, North-East-Down wind, and attitude
% solution.
%
% INPUTS:
%   V_NED: Nx3 vector of inertial velocities (ft/s)
%   V_w_NED: 3x1 vector of wind velocities (ft/s)
%   phi: Nx1 vector of roll attitude (deg)
%   theta: Nx1 vector of pitch attitude (deg)
%   psi: Nx1 vector of heading attitude (deg)
% 
% OUTPUTS:
%   uvw: Nx3 vector of body fixed x-y-z air velocities (ft/s)
%   Vab: Nx3 vector of V-alpha-beta [ft/s, deg, deg]
%   
% Sam Jaeger
% jaege246@umn.edu
% 8/5/2026

function [uvw, Vab] = GPS_SAD(V_NED, V_w_NED, phi, theta, psi)
    %N = length(phi);
    
    % check dimensions of inputs-----
    sz_Vned = size(V_NED);
    sz_Vw = size(V_w_NED);

    if iscolumn(theta) == false
        theta = theta';
    end
    sz_theta = size(theta);
    N = sz_theta(1);

    if iscolumn(psi) == false 
        psi = psi';
    end
    sz_psi = size(psi);

    if iscolumn(phi) == false
        phi = phi';
    end
    sz_phi = size(phi);

    if sz_Vned(1) ~= N || sz_Vned(2) ~= 3
        error('V_NED must be Nx3 matrix')
    elseif sz_Vw(1) ~= 3 || sz_Vw(2) ~= 1
        error('wind vector must be 3x1')
    end

    if sz_psi(1) ~= N
        error('psi must be a column vector of length N')
    elseif  sz_phi(1) ~= N
        error('phi must be a column vector of length N')
    end

    % calculate DCM and map velocity to body frame
    uvw = zeros(3,N);
    V = zeros(N,1);
    alpha = zeros(N,1);
    beta = zeros(N,1);
    for ii=1:N
        DCM(:,1) = [cosd(theta(ii))*cosd(psi(ii)); cosd(theta(ii))*sind(psi(ii)); -sind(theta(ii))];
        DCM(:,2) = [sind(phi(ii))*sind(theta(ii))*cosd(psi(ii)) - cosd(phi(ii))*sind(psi(ii)); sind(phi(ii))*sind(theta(ii))*sind(psi(ii)) + cosd(phi(ii))*cosd(psi(ii)); sind(phi(ii))*cosd(theta(ii))];
        DCM(:,3) = [cosd(phi(ii))*sind(theta(ii))*cosd(psi(ii)) + sind(phi(ii))*sind(psi(ii)); cosd(phi(ii))*sind(theta(ii))*sind(psi(ii)) - sind(phi(ii))*cosd(psi(ii)); cosd(phi(ii))*cosd(theta(ii))];
        
        uvw(:,ii) = DCM'*(V_NED(ii,:)'-V_w_NED);
        V(ii) = norm(uvw(:,ii));
    
        alpha(ii) = atan2d(uvw(3,ii),uvw(1,ii)); % degrees
        beta(ii) = asind(uvw(2,ii)./V(ii)); % degrees
    end
    uvw = uvw';
    Vab = [V, alpha, beta];
end