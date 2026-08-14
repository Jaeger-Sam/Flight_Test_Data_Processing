% This script formats and processes maneuvers prior to regression analysis. 
%
% Sam Jaeger
% jaege246@umn.edu
% 8/12/2026

% initalize regression variables...........................................
C_D = [];
C_Yw = [];
C_L = [];
C_l = [];
C_m = [];
C_n = [];
u_inf = [];
alpha = [];
beta = [];
delta_e = [];
delta_a =[];
delta_r = [];
delta_f = [];
p = []; 
q = [];
r = [];

% initalize validation variables...........................................
C_D_v = [];
C_Yw_v = [];
C_L_v = [];
C_l_v = [];
C_m_v = [];
C_n_v = [];
u_inf_v = [];
alpha_v = [];
beta_v = [];
delta_e_v = [];
delta_a_v =[];
delta_r_v = [];
delta_f_v = [];
p_v = []; 
q_v = [];
r_v = [];


% loop over jth maneuver
maneuvers = [maneuver_to_include, maneuver_to_validate];
i_maneuver = zeros(length(maneuvers),1); % index of when maneuver changes
for kk=1:length(maneuvers)
jj = maneuvers(kk);

disp('-------------------------------------------------')
disp(append('Begin Processing Manuever jj = ',num2str(jj),'. Maneuver = ',maneuver_label(jj)))

if thist_align(kk) ==1 % GPS time SYNC
    data_FMUR = process_FMUR_data(test_data, i_FMUR_se_0(jj,:), cal, altimeter, temperature);
else % GYRO SYNC
    data_FMUR = process_FMUR_data(test_data, i_FMUR_se(jj,:), cal, altimeter, temperature);
end

% plot FMUR data...........................................................
if plt_FMUR_data == true
    tf_save_fig = false;
    if thist_align == 1 % GPS time
        i_FMUR_s = i_FMUR_se_0(jj,1);
        i_FMUR_e = i_FMUR_se_0(jj,2);
    else % GYRO SYNC
        i_FMUR_s = i_FMUR_se(jj,1);
        i_FMUR_e = i_FMUR_se(jj,2);
    end
    flight_test_plots_FMUR(test_data.FMUR,cal,tf_save_fig,i_FMUR_s,i_FMUR_e);
    disp('Check FMU-R plots and hit enter.')
    pause
end

% Process n580 data........................................................
if thist_align == 1
    data_n580 = process_n580_data(test_data, t_GNSS_se(1),t_GNSS_se(2),i_n580_raw_se_0(jj,:),i_n580_filt_se_0(jj,:));
    if plt_n580_data == true
        flight_test_plots_n580(test_data.n580,tf_save_fig,i_n580_raw_se_0(jj,:),i_n580_filt_se_0(jj,:))
        disp('Check n580 plots and hit enter.')
        pause
    end
else
    data_n580 = process_n580_data(test_data, t_GNSS_se(1),t_GNSS_se(2),i_n580_raw_se(jj,:),i_n580_filt_se(jj,:));
    if plt_n580_data == true
        flight_test_plots_n580(test_data.n580,tf_save_fig,i_n580_raw_se(jj,:),i_n580_filt_se(jj,:))
        disp('Check n580 plots and hit enter.')
        pause
    end
end
close all;


% put data into columns ...................................................
if IMU_n580 ==1
    ax0 = LP_15smooth(data_n580.ax) - mean(b_a.n580(1,:));
    ay0 = LP_15smooth(data_n580.ay) - mean(b_a.n580(2,:));
    az0 = LP_15smooth(data_n580.az) - mean(b_a.n580(3,:));
    p0 = data_n580.p - mean(b_g.n580(1,:));
    q0 = data_n580.q - mean(b_g.n580(2,:));
    r0 = data_n580.r - mean(b_g.n580(3,:));
else
    ax0 = LP_15smooth(data_FMUR.ax) - mean(b_a.FMUR(1,:));
    ay0 = LP_15smooth(data_FMUR.ay) - mean(b_a.FMUR(2,:));
    az0 = LP_15smooth(data_FMUR.az) - mean(b_a.FMUR(3,:));
    p0 = data_FMUR.pqr(:,1) - mean(b_g.FMUR(1,:));
    q0 = data_FMUR.pqr(:,2) - mean(b_g.FMUR(2,:));
    r0 = data_FMUR.pqr(:,3) - mean(b_g.FMUR(3,:));
end

u_pitot = LP_15smooth(data_FMUR.u_pitot);
alpha_wt = LP_15smooth(data_FMUR.alpha);
beta_wt = LP_15smooth(data_FMUR.beta);
delta_e0 = LP_15smooth(data_FMUR.delta_e);
delta_a0 = LP_15smooth(data_FMUR.delta_a);
delta_r0 = LP_15smooth(data_FMUR.delta_r);
delta_f0 = LP_15smooth(data_FMUR.delta_f);
rpm = LP_15smooth( hampel(data_FMUR.n*60,15) );
rho_inf = data_FMUR.rho_amb;

% translate pitot to CG
y_pitot = cal.pitot_xyx_CG(2);
z_pitot = cal.pitot_xyx_CG(3);
x_wt = cal.AB_xyz_CG(1);
y_wt = cal.AB_xyz_CG(2);
z_wt = cal.AB_xyz_CG(3);
[u_inf0,alpha0,beta0] = air_data_tip_to_CG(u_pitot,alpha_wt,beta_wt,p0*pi/180,q0*pi/180,r0*pi/180,y_pitot,z_pitot,x_wt,y_wt,z_wt,plt_wt_cg_thist);
if plt_wt_cg_thist == true
    disp('Inspect air data transformation plot and hit enter.')
    pause
end

% GPS air data.............................................................
% load in n580 data (Velocity + attitude)
V_NED = data_n580.V; % NED velocity data
phi = data_n580.phi;% roll
theta = data_n580.theta; % pitch
psi = data_n580.psi; % yaw

% hard code in wind properties
V_w_NED = [V_w_mag*cosd(psi_wind); V_w_mag*sind(psi_wind); 0];

[uvw_GPS, Vab_GPS] = GPS_SAD(V_NED, V_w_NED, phi, theta, psi, plt_n580_alpha_beta_thist);
if plt_n580_alpha_beta_thist == true
    disp('Inspect synthetic air data solution and hit enter.')
    pause
end

% calculate thrust.........................................................
[T,CT,J] = calc_thrust(rpm,u_inf0,alpha0,beta0,rho_inf,p_CT,d,plt_thrust);
if plt_thrust == true
    disp('Inspect thrust plot and hit enter.')
    pause
end

% estimate aero coefs......................................................

% aerodynamic force coefficients...
b_a_xyz = [0;0;0]; % accel bias (bias already removed from above)
[C_s, C_b] = generate_aero_force_coef(ax0, ay0, az0, u_inf0, alpha0, beta0, T, rho_inf, W, Sw, b_a_xyz, plt_aero_force_coefs);
if plt_aero_force_coefs == true
    disp('Inspect force coef plot and hit enter.')
    pause
end

% aerodynamic moment coefficients...
b_g_xyz = [0;0;0]; % gyro bias (bias already removed from above)
M_prop = zeros(length(T),3);
M_prop(:,2) = z_prop*T;
[C_lmn]= generate_aero_moment_coef(p0,q0,r0, u_inf0,alpha0,beta0, rho_inf, Ixx,Iyy,Izz,Ixz, Sw,cbw,bw, b_g_xyz,plt_aero_moment_coefs,M_prop,method,f_max);
if plt_aero_moment_coefs == true
    disp('Inspect moment coef plot and hit enter.')
    pause
end

% put variables into column vectors........................................
if jj == maneuver_to_validate
    disp(append('validation maneuver jj = ',num2str(jj)))
    N = length(alpha0);
    %i_maneuver(jj) = length(C_D_v) + N - (ic_s+ic_e);
    C_D_v = [C_D_v; C_s(ic_s:(N-ic_e),1)];
    C_Yw_v = [C_Yw_v; C_s(ic_s:(N-ic_e),2)];
    C_L_v = [C_L_v; C_s(ic_s:(N-ic_e),3)];
    C_l_v = [C_l_v; C_lmn(ic_s:(N-ic_e),1)];
    C_m_v = [C_m_v; C_lmn(ic_s:(N-ic_e),2)];
    C_n_v = [C_n_v; C_lmn(ic_s:(N-ic_e),3)];
    u_inf_v = [u_inf_v; u_inf0(ic_s:(N-ic_e))];
    alpha_v = [alpha_v; alpha0(ic_s:(N-ic_e))];
    beta_v = [beta_v; beta0(ic_s:(N-ic_e))];
    delta_e_v = [delta_e_v; delta_e0(ic_s:(N-ic_e))];
    delta_a_v =[delta_a_v; delta_a0(ic_s:(N-ic_e))];
    delta_r_v = [delta_r_v; delta_r0(ic_s:(N-ic_e))];
    delta_f_v = [delta_f_v; delta_f0(ic_s:(N-ic_e))];
    p_v = [p_v; p0(ic_s:(N-ic_e))]; 
    q_v = [q_v; q0(ic_s:(N-ic_e))];
    r_v = [r_v; r0(ic_s:(N-ic_e))];
else
    N = length(alpha0);
    i_maneuver(jj) = length(C_D) + N - (ic_s+ic_e);
    C_D = [C_D; C_s(ic_s:(N-ic_e),1)];
    C_Yw = [C_Yw; C_s(ic_s:(N-ic_e),2)];
    C_L = [C_L; C_s(ic_s:(N-ic_e),3)];
    C_l = [C_l; C_lmn(ic_s:(N-ic_e),1)];
    C_m = [C_m; C_lmn(ic_s:(N-ic_e),2)];
    C_n = [C_n; C_lmn(ic_s:(N-ic_e),3)];
    u_inf = [u_inf; u_inf0(ic_s:(N-ic_e))];
    alpha = [alpha; alpha0(ic_s:(N-ic_e))];
    beta = [beta; beta0(ic_s:(N-ic_e))];
    delta_e = [delta_e; delta_e0(ic_s:(N-ic_e))];
    delta_a =[delta_a; delta_a0(ic_s:(N-ic_e))];
    delta_r = [delta_r; delta_r0(ic_s:(N-ic_e))];
    delta_f = [delta_f; delta_f0(ic_s:(N-ic_e))];
    p = [p; p0(ic_s:(N-ic_e))]; 
    q = [q; q0(ic_s:(N-ic_e))];
    r = [r; r0(ic_s:(N-ic_e))];
end

close all;
end