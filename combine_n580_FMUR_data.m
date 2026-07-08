%  combine_n580_FMUR_data.fcn combines the FMUR and n580 data into a single
%  data structure that is time synced (provided that GPS time syncing is
%  possible between the two systems). 
%
%  data_int = combine_n580_FMUR_data(data_FMUR,data_n580)
%
% INPUTS:
%   data_FMUR: data structure from process_FMUR_data.fcn
%   data_n580: data structure from process_n580_data.fcn
%
% OUTPUTS:
%   data_int: combined data structure with FMUR and n580 data consisent
%       with the simulator functions.
%
% Sam Jaeger
% jaege246@umn.edu
% 7/3/2026

function data_int = combine_n580_FMUR_data(data_FMUR,data_n580)
    
    % data cropping timing
    data_int.t_GNSS_start = data_FMUR.t_GNSS_start;
    data_int.t_GNSS_end = data_FMUR.t_GNSS_end;

    % data cropping indexing
    data_int.i_FMUR_se = [data_FMUR.index_crop(1),  data_FMUR.index_crop(2)];
    data_int.i_n580_raw_se = data_n580.i_n580_raw_se;
    data_int.i_n580_filt_se = data_n580.i_n580_filt_se;
   

    % altitudes (ft) from FMUR
    data_int.altitude.FMUR.msl= data_FMUR.altitude.g_msl; % geopotential
    data_int.altitude.FMUR.wgs84 = data_FMUR.altitude.g_wgs84; % geopotential
   
    % chapter 3 Pitot Statics and the Standard Atmosphere, Russell Erb
    data_int.altitude.FMUR.p = data_FMUR.altitude.p; % pressure altitude
    data_int.altitude.FMUR.rho = data_FMUR.altitude.rho;% density altitude

    % altitude (ft) from n580
    data_int.altitude.n580.msl = data_n580.PHI_PSI_H(:,3);
    
    % control input
    data_int.control_vec = data_FMUR.control_vec;

    % velocity - total airspeed (ft/s)
    data_int.Velocity = data_FMUR.Velocity;

    % alpha (angle of attack deg)
    data_int.alpha = data_FMUR.alpha;
    data_int.alpha_CG = data_FMUR.alpha_CG;
    
    % beta / beta_f (flank angle deg)
    data_int.beta_f = data_FMUR.beta_f;
    data_int.beta_f_CG = data_FMUR.beta_f_CG;

    % beta (angle of sideslip deg)
    data_int.beta = data_FMUR.beta;
    data_int.beta_CG = data_FMUR.beta_CG;

    % alpha_dot
    data_int.alpha_dot = data_FMUR.alpha_dot;

    % beta_f_dot
    data_int.beta_f_dot = data_FMUR.beta_f_dot;

    % beta_dot
    data_int.beta_dot = data_FMUR.beta_dot;

    % time
    %data_int.t_out = data_FMUR.t_out;
    data_int.t_out = data_n580.t_filt; % use n580 GPS time for everything

    % X_out (state vector)
    data_int.X_out = [];

    % u (ft/s) measured at pitot
    %   translate u velocity to CG
    data_int.u_pitot = data_FMUR.u_pitot;
    
    data_int.u_NB = data_FMUR.u_NB;

    % {u,v,w} (ft/s) (at vehicle center)
    data_int.uvw = data_FMUR.uvw;
    
    % inertial velocity (ft/s)
    data_int.V.FMUR = data_FMUR.V;
    data_int.V.n580 = data_n580.V;
    data_int.Velocity_i.FMUR = data_FMUR.Velocity_i;
    data_int.Velocity_i.n580 = data_n580.Velocity_i;

    % {p,q,r} (deg/s) 
    data_int.pqr.FMUR = data_FMUR.pqr;
    data_int.pqr.n580 = [data_n580.p, data_n580.q, data_n580.r];

    % {x,y,z} (ft)
    data_int.XYZ.FMUR = data_FMUR.XYZ;
    data_int.XYZ.n580 = [data_n580.northing, data_n580.easting, -data_n580.PHI_PSI_H(:,3)];

    % {e0,ex,ey,ez}
    %data_int.es_out = [];

    % phi_theta_psi (deg)
    phi_theta_psi = [data_n580.phi, data_n580.theta, data_n580.psi];

    %data_FMUR.es_out = quats_from_attitude(phi_theta_psi);
    data_int.es_out.FMUR = data_FMUR.es_out;
    data_int.es_out.n580 = quats_from_attitude(phi_theta_psi*pi/180);
    data_int.phi_theta_psi.FMUR = data_FMUR.phi_theta_psi; % deg
    data_int.phi_theta_psi.n580 = phi_theta_psi;
    
    % use air data from FMUR, ang vel. & position & attitude from n580
    % size(data_int.uvw)
    % size(data_int.pqr.n580)
    % size(data_int.XYZ.n580)
    % size(data_int.es_out.n580)
    % size(data_int.t_out)
    data_int.X_out = [data_int.uvw, data_int.pqr.n580, data_int.XYZ.n580, data_int.es_out.n580];

    % phi (deg)
    data_int.phi = phi_theta_psi(:,1);

    % theta (deg)
    data_int.theta = phi_theta_psi(:,2);

    % psi (deg)
    data_int.psi = phi_theta_psi(:,3);

    % gamma (deg)
    data_int.gamma.FMUR = data_FMUR.gamma; 
    % need to check this ...
    data_int.gamma.n580 = asind( -sind(data_int.alpha_CG).*sind(data_int.theta).*sind(data_int.phi) + ...
        cosd(data_int.beta_CG).*( cosd(data_int.alpha_CG).*sind(data_int.theta) - ...
        sind(data_int.alpha_CG).*cosd(data_int.theta).*cosd(data_int.phi) ));

    % delta_T
    data_FMUR.delta_T = [];

    % delta_e
    data_int.delta_e = data_FMUR.delta_e;

    % delta_a
    data_int.delta_a_r = data_FMUR.delta_a_r;
    data_int.delta_a_l = data_FMUR.delta_a_l;
    data_int.delta_a = data_FMUR.delta_a;

    % delta_r
    data_int.delta_r = data_FMUR.delta_r ;

    % delta_f
    data_int.delta_f_r = data_FMUR.delta_f_r;
    data_int.delta_f_l = data_FMUR.delta_f_l;
    data_int.delta_f = data_FMUR.delta_f;

    % lat-long-altitude
    data_int.PHI_PSI_H.FMUR = data_FMUR.PHI_PSI_H;
    data_int.PHI_PSI_H.n580 = data_n580.PHI_PSI_H;

    % rho_inf (slugs / ft^3)
    data_int.rho = data_FMUR.rho;
    data_int.rho_amb = data_FMUR.rho_amb;

    % T_inf (F)
    data_int.T = data_FMUR.T;
    data_int.T_amb = data_FMUR.T_amb;

    % p (lb / ft^2)
    data_int.p = data_FMUR.p; %  lb/ft^2
    data_int.p_amb = data_FMUR.p_amb; %  lb/ft^2

    % a_inf (ft/s)
    data_int.a = data_FMUR.a;

    % nu_inf
    data_int.nu = data_FMUR.nu;

    % g_inf
    data_FMUR.g = [];

    % Mach
    data_int.Mach = data_FMUR.Mach;

    % Reynolds
    data_int.Reynolds = data_FMUR.Reynolds;

    % qbar
    data_int.qbar = data_FMUR.qbar;

    % ax,ay,az 
    data_int.ax.FMUR = data_FMUR.ax;
    data_int.ay.FMUR = data_FMUR.ay;
    data_int.az.FMUR = data_FMUR.az;
    data_int.ax.n580 = data_n580.ax;
    data_int.ay.n580 = data_n580.ay;
    data_int.az.n580 = data_n580.az;

    % p_b non dim roll
    data_int.p_b = data_FMUR.p_b;

    % q_b
    data_int.q_b = data_FMUR.q_b;

    % r_b
    data_int.r_b = data_FMUR.r_b;

    % rpm (rev/min)
    data_int.rpm = data_FMUR.rpm;

    % n (rev/s)
    data_int.n = data_FMUR.n;

    % J
    data_int.J = data_FMUR.J;
end