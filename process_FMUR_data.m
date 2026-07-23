% process_flight_test_data.fcn processes the flight test data structure
% outputted by read_flight_test_data.fcn. Converts data into US units and
% uses the RK4 output as the returned data structure. Will ask user if they
% want to crop the data. Computes various altitudes including pressure and
% density following the fromulation in Chapter 3 of Pitot Statics and the
% Standard Atmosphere by Russel Erb (USAF TPS Publication).
% 
% data_FMUR = process_FMUR_data(test_data, i_FMUR_se, cal, altimeter, temperature)
%
% INPUTS:
%   test_data: data strcture from read_flight_test_data.fcn
%   i_FMUR_se: 2 element vector of starting and ending cropping indices.
%   cal: data structure of calibrations for the flight vehicle
%       .p_alpha: polynomial of calibration of the angle of attack pot.
%           (volts to deg)
%       .p_beta_f: polynomial of calibration of the angle of sideslip 
%           (flank angle) pot. (volts to deg)
%       .p_delta_e: polynomial of calibration of the elevator pot.
%           (volts to deg)
%       .p_delta_a_r: polynomial of calibration of the right aileron pot.
%           (volts to deg)
%       .p_delta_a_l: polynomial of calibration of the left aileron pot.
%           (volts to deg)
%       .p_delta_r: polynomial of calibration of the rudder pot.
%           (volts to deg)
%       .p_delta_f_r: polynomial of calibration of the right flap pot.
%           (volts to deg)
%       .p_delta_f_l: polynomial of calibration of the left flap pot.
%           (volts to deg)
%       .c_b_w: mean aero chord of main wing in ft for Re calculation
%       .b_w: wingspan of main wing in ft for non dim ang rate calcs
%       .d: diameter of prop in ft for advance ratio calculation
%       .FMUR_xyz_CG  FMUR w.r.t cg location (ft)
%       .AB_xyz_CG  alpha beta sensor w.r.t cg (ft)
%       .pitot_xyx_CG: pitot probe w.r.t cg (ft)
%       .pres_stat_bias: static pressure sensor bias (lb/ft^2)
%       .lam_Dp: differential pressure sensor scale factor error
%       .b_Dp: differential pressure sensor bias (lb/ft^2)
%   altimeter: altimeter setting for the day (in Hg)
%   temperature: ambient temperature for flight test (F)
%
% OUTPUTS:
%   data_FMUR: data structure of processed data.
%
% Sam Jaeger
% jaege246@umn.edu
% 10/28/2025
%   Revised: 1/26/2026
%   Revised: 7/2/2026

function data_FMUR = process_FMUR_data(test_data, i_FMUR_se, cal, altimeter, temperature)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Integrate Data FMU-R into single data structure
    i_FMUR_s = i_FMUR_se(1);
    i_FMUR_e = i_FMUR_se(2);
    
    data_FMUR.t_GNSS_start = test_data.FMUR.sys.time_gnss_s(i_FMUR_s);
    data_FMUR.t_GNSS_end = test_data.FMUR.sys.time_gnss_s(i_FMUR_e);
    data_FMUR.index_crop(1) = i_FMUR_s;
    data_FMUR.index_crop(2) = i_FMUR_e;

    data_FMUR.i_GPS_loss = [];
    data_FMUR.i_GPS_gain = [];

    if test_data.FMUR.sys.time_gnss_s(i_FMUR_s) > 1e3
        % use GNSS time if there is a GPS lock
        GPS_lock = true;
        t_FMUR = test_data.FMUR.sys.time_gnss_s(i_FMUR_s:i_FMUR_e) - test_data.FMUR.sys.time_gnss_s(i_FMUR_s);
    else 
        % use system time if no GPS lock
        GPS_lock = false;
        i_GPS_loss = i_FMUR_s;
        t_FMUR = test_data.FMUR.sys.time_s(i_FMUR_s:i_FMUR_e) - test_data.FMUR.sys.time_s(i_FMUR_s);
    end
    count =1;
    for ii=(i_FMUR_s+1):i_FMUR_e
        if abs(test_data.FMUR.sys.time_gnss_s(ii)-test_data.FMUR.sys.time_gnss_s(ii-1)) > 1e3 && GPS_lock == true
            i_GPS_loss = ii;
            GPS_lock = false;
            count_gps_loss = 1;
            data_FMUR.i_GPS_loss = i_GPS_loss;
        elseif abs(test_data.FMUR.sys.time_gnss_s(ii)-test_data.FMUR.sys.time_gnss_s(ii-1)) > 1e3 && GPS_lock == false
            i_GPS_gain = ii;
            t_FMUR(count) = count_gps_loss*0.01 + t_gps_loss;
            GPS_lock = true;
            data_FMUR.i_GPS_gain = i_GPS_gain;
        end

        if GPS_lock == false
            % replace time with gps time when the signal was lost (relying
            %  on system time)

            t_gps_loss = t_FMUR((i_GPS_loss-i_FMUR_s)-1);
            t_FMUR(count) = count_gps_loss*0.01 + t_gps_loss;
            count_gps_loss = count_gps_loss + 1;
        end
        count=count+1;
    end

    % for day (ideal gas) in SI units
    R = 287; %J/ kg K
    p_amb = altimeter/0.00029529983071445; % Pa
    T_amb = (temperature + 459.67)*5/9; % K
    rho_amb = p_amb/R/T_amb*0.0019403203; %kg/m^3 convert to slugs/ft^3

    % altitudes (m)
    altitude.g_msl = test_data.FMUR.gnss.alt_msl_m(i_FMUR_s:i_FMUR_e)*3.28084; % geopotential
    altitude.g_wgs84 = test_data.FMUR.gnss.alt_wgs84_m(i_FMUR_s:i_FMUR_e)*3.28084; % geopotential
   
    % pressure, temperature, density ratios
    [~,T_SL,p_SL,~,~] = ATMOS(0,'SI');
    delta_rat = test_data.FMUR.pres.static.pres_pa(i_FMUR_s:i_FMUR_e)/p_SL - cal.pres_stat_bias/47.880258888889/p_SL;    
    theta_rat = T_amb/T_SL;
    sigma_rat = delta_rat./theta_rat;
   
    
    % chapter 3 Pitot Statics and the Standard Atmosphere, Russell Erb
    altitude.p = (1 - (delta_rat.^(1/5.2559)))/(6.87559*10^(-6)); % pressure
    altitude.rho = (1 - (sigma_rat.^(1/4.2559)))/(6.87559*10^(-6));% density
    
    data_FMUR.altitude = altitude;

    % control input
    data_FMUR.control_vec = test_data.FMUR.vms.pwm_cmd(i_FMUR_s:i_FMUR_e,:);

    % velocity (ft/s)
    data_FMUR.Velocity = [];

    % alpha (angle of attack deg)
    alpha = polyval(cal.p_alpha,test_data.FMUR.adc_volt((i_FMUR_s:i_FMUR_e),2));
    data_FMUR.alpha = alpha;
    data_FMUR.alpha_CG = [];
    
    % beta / beta_f (flank angle deg)
    beta = polyval(cal.p_beta_f,test_data.FMUR.adc_volt((i_FMUR_s:i_FMUR_e),1));
    beta_f = atan2d( tand(beta),cosd(alpha));
    data_FMUR.beta_f = beta_f;
    data_FMUR.beta_f_CG = [];

    % beta (angle of sideslip deg)
    data_FMUR.beta = beta;
    data_FMUR.beta_CG = [];

    % alpha_dot
    data_FMUR.alpha_dot = [];

    % beta_f_dot
    data_FMUR.beta_f_dot = [];

    % beta_dot
    data_FMUR.beta_dot = [];

    % time
    data_FMUR.t_out = t_FMUR;

    % X_out (state vector)
    data_FMUR.X_out = [];

    % pqr for translating u v w to cg
    %   gyro data backwards
    pqr = -test_data.FMUR.imu.gyro_radps(i_FMUR_s:i_FMUR_e,:);

    % u (ft/s) measured at pitot
    rho = 0.0023769.*sigma_rat; % estimate for density
    %qbar = (test_data.FMUR.pres.diff.pres_pa(i_FMUR_s:i_FMUR_e) - cal.pres_diff_bias)*0.020885434273039; %lb/ft^2
    %u_pitot = sqrt(2*qbar./rho) + cal.u_bias;
    u_pitot = calibrated_airspeed(test_data.FMUR.pres.diff.pres_pa(i_FMUR_s:i_FMUR_e)*0.020885434273039, cal.lam_Dp, cal.b_Dp);
    
    for ii=1:length(u_pitot)
        if isreal(u_pitot(ii)) == false
            u_pitot(ii)=0;
        end
    end
    % translate u velocity to CG
    u = u_pitot + pqr(:,3)*cal.pitot_xyx_CG(2) - pqr(:,2)*cal.pitot_xyx_CG(3);
    data_FMUR.u_pitot = u_pitot;
    
    u_NB = u_pitot + pqr(:,3)*(cal.pitot_xyx_CG(2) - cal.AB_xyz_CG(2)) - pqr(:,2)*(cal.pitot_xyx_CG(3) - cal.AB_xyz_CG(3));
    data_FMUR.u_NB = u_NB;

    % {u,v,w} (ft/s) (in vehicle center)
    beta_f_CG = atand((u_NB.*tand(beta_f) + pqr(:,1)*cal.AB_xyz_CG(3) - pqr(:,3)*cal.AB_xyz_CG(1))./u);
    alpha_CG = atand((u_NB.*tand(alpha) + pqr(:,2)*cal.AB_xyz_CG(1) - pqr(:,1)*cal.AB_xyz_CG(2))./u);
    data_FMUR.beta_f_CG = beta_f_CG;
    data_FMUR.alpha_CG = alpha_CG;
    data_FMUR.beta_CG = atand(tand(data_FMUR.beta_f_CG).*cosd(data_FMUR.alpha_CG));

    v = u.*tand(data_FMUR.beta_f_CG);
    w = u.*tand(data_FMUR.alpha_CG);
    data_FMUR.uvw = [u,v,w];
    data_FMUR.Velocity = vecnorm(data_FMUR.uvw,2,2);
    data_FMUR.V =  test_data.FMUR.gnss.ned_vel_mps(i_FMUR_s:i_FMUR_e,:)*3.28084;
    data_FMUR.Velocity_i = vecnorm( test_data.FMUR.gnss.ned_vel_mps(i_FMUR_s:i_FMUR_e,:),2,2)*3.28084;

    % {p,q,r} (deg/s) 
    data_FMUR.pqr = pqr*180/pi;

    % {x,y,z} (ft)
    data_FMUR.XYZ = test_data.FMUR.nav.ned.pos_m(i_FMUR_s:i_FMUR_e,:)*3.28084;

    % {e0,ex,ey,ez}
    %data_int.es_out = [];

    % phi_theta_psi (deg)
    phi_theta_psi = [test_data.FMUR.nav.roll_rad(i_FMUR_s:i_FMUR_e,:),...
        test_data.FMUR.nav.pitch_rad(i_FMUR_s:i_FMUR_e,:),...
        test_data.FMUR.nav.heading_rad(i_FMUR_s:i_FMUR_e,:)]; % rad
    
    data_FMUR.es_out = quats_from_attitude(phi_theta_psi);
    data_FMUR.phi_theta_psi = phi_theta_psi*180/pi; % deg
    
    data_FMUR.X_out = [data_FMUR.uvw, data_FMUR.pqr, data_FMUR.XYZ, data_FMUR.es_out];

    % phi (deg)
    data_FMUR.phi = phi_theta_psi(:,1)*180/pi;

    % theta (deg)
    data_FMUR.theta = phi_theta_psi(:,2)*180/pi;

    % psi (deg)
    data_FMUR.psi = phi_theta_psi(:,3)*180/pi;

    % gamma (deg)
    data_FMUR.gamma = test_data.FMUR.nav.flight_path_rad(i_FMUR_s:i_FMUR_e,:)*180/pi; 

    % delta_T
    data_FMUR.delta_T = [];

    % delta_e
    data_FMUR.delta_e = polyval(cal.p_delta_e,test_data.FMUR.adc_volt(i_FMUR_s:i_FMUR_e,3));

    % delta_a
    data_FMUR.delta_a_r = polyval(cal.p_delta_a_r,test_data.FMUR.adc_volt(i_FMUR_s:i_FMUR_e,5));
    data_FMUR.delta_a_l = polyval(cal.p_delta_a_l,test_data.FMUR.adc_volt(i_FMUR_s:i_FMUR_e,6));
    data_FMUR.delta_a = (data_FMUR.delta_a_r + data_FMUR.delta_a_l)/2;

    % delta_r
    data_FMUR.delta_r = polyval(cal.p_delta_r, test_data.FMUR.adc_volt(i_FMUR_s:i_FMUR_e,4));

    % delta_f
    data_FMUR.delta_f_r = polyval(cal.p_delta_f_r,test_data.FMUR.adc_volt(i_FMUR_s:i_FMUR_e,8));
    data_FMUR.delta_f_l = polyval(cal.p_delta_f_l,test_data.FMUR.adc_volt(i_FMUR_s:i_FMUR_e,7));
    data_FMUR.delta_f = (data_FMUR.delta_f_r + data_FMUR.delta_f_l)/2;

    % lat-long-altitude
    PHI_PSI_H = [test_data.FMUR.gnss.lat_rad(i_FMUR_s:i_FMUR_e),...
        test_data.FMUR.gnss.lon_rad(i_FMUR_s:i_FMUR_e),...
        test_data.FMUR.gnss.alt_wgs84_m(i_FMUR_s:i_FMUR_e)*3.28084];
    data_FMUR.PHI_PSI_H = PHI_PSI_H;

    % rho_inf (slugs / ft^3)
    data_FMUR.rho = rho;
    data_FMUR.rho_amb = rho_amb;

    % T_inf (F)
    data_FMUR.T = theta_rat*59;%(T_SL - 273.15)*(9/5);
    data_FMUR.T_amb = temperature;

    % p (lb / ft^2)
    data_FMUR.p = delta_rat*p_SL/47.880258888889; % convert to lb/ft^2
    data_FMUR.p_amb = p_amb/47.880258888889; % convert to lb/ft^2

    % a_inf (ft/s)
    data_FMUR.a = sqrt(1.4*R*T_amb)*3.28084;

    % nu_inf
    mu = sutherland(data_FMUR.T+459.67,'US'); % in R
    data_FMUR.nu = mu./rho;

    % g_inf
    data_FMUR.g = [];

    % Mach
    data_FMUR.Mach = u./data_FMUR.a;

    % Reynolds
    data_FMUR.Reynolds = rho.*cal.c_b_w.*u./mu;

    % qbar
    %data_int.qbar = qbar;
    data_FMUR.qbar = 0.5.*rho.*u.^2;

    % ax,ay,az 
    %   FMUR accel backwards
    data_FMUR.ax = -test_data.FMUR.imu.accel_mps2(i_FMUR_s:i_FMUR_e,1)*3.28084;
    data_FMUR.ay = -test_data.FMUR.imu.accel_mps2(i_FMUR_s:i_FMUR_e,2)*3.28084;
    data_FMUR.az = -test_data.FMUR.imu.accel_mps2(i_FMUR_s:i_FMUR_e,3)*3.28084;

    % p_b non dim roll
    data_FMUR.p_b = data_FMUR.pqr(:,1)*cal.b_w./2./data_FMUR.Velocity;

    % q_b
    data_FMUR.q_b = data_FMUR.pqr(:,2)*cal.c_b_w./2./data_FMUR.Velocity;

    % r_b
    data_FMUR.r_b = data_FMUR.pqr(:,3)*cal.b_w./2./data_FMUR.Velocity;

    % rpm (rev/min)
    data_FMUR.rpm = test_data.FMUR.pwr.rpm(i_FMUR_s:i_FMUR_e);

    % n (rev/s)
    data_FMUR.n = test_data.FMUR.pwr.rpm(i_FMUR_s:i_FMUR_e)/60;

    % J
    data_FMUR.J = u./data_FMUR.n./cal.d;

    % remove divide by zero parameters
    for ii=1:length(data_FMUR.Velocity)
        if data_FMUR.Velocity(ii) <=0.1
            data_FMUR.p_b(ii) = 0;
            data_FMUR.q_b(ii) = 0;
            data_FMUR.r_b(ii) = 0;
            data_FMUR.J(ii) =0;
        elseif data_FMUR.n(ii) <= 1
            data_FMUR.J(ii) = 0;
        end
    end
end