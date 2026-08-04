% Attitude animation script
% 
% This script formats the FMUR and n580 data into the appropriate form for
% the aircraft_3d_animation.fcn. This will uses the IBIS 3d model for the
% animation. For more information on the animation code refer to: 
% https://github.com/Ro3code/aircraft_3d_animation
%
% Sam Jaeger
% 7/28/2026

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Load Data

disp('Select .mat datafile that has the time synced FMU-R and n580 data...')
[flight_file,flight_floc] = uigetfile('*.mat');
load(append(flight_floc,'\',flight_file));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Crop Data (if needed)

[t_GNSS_se,i_FMUR_se, i_n580_raw_se,i_n580_filt_se,maneuver_label] = crop_flight_test_data(test_data);


altimeter = 29.92; % in Hg
temperature = 82; % deg F

disp('Select .mat data file for calibration.')
[cal_file,cal_floc] = uigetfile('*.mat');
load(append(cal_floc,'\',cal_file),'cal');
    
%     disp('Input altimeter setting and temperature of the day.')
%     altimeter =   input('altimeter (in Hg):    ');
%     temperature = input('temperature (F):      ');

% need load in cal file
data_FMUR = process_FMUR_data(test_data, i_FMUR_se, cal, altimeter, temperature);
data_n580 = process_n580_data(test_data, t_GNSS_se(1),t_GNSS_se(2),i_n580_raw_se,i_n580_filt_se);
data_int = combine_n580_FMUR_data(data_FMUR,data_n580);
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% data structures and timing
%test_data = TD;

data_int = test_data.data_int;
data_n580 = test_data.data_n580;
data_FMUR = test_data.data_FMUR;
%%
%t = data_int.t_out;
t = data_FMUR.t_out;
%t = data_n580.t_raw;
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Check Attitude 

% phi = data_int.phi;
% theta = data_int.theta;
% psi = data_int.psi; % some of the data of the 6/30/26 flight is negative

phi = data_n580.phi;
theta = data_n580.theta;
psi = data_n580.psi + 180;
for ii =1:length(psi)
    if psi(ii) > 180
        psi(ii) = psi(ii) -360;
    end
end

% phi = TD.data_n580.phi;
% theta = TD.data_n580.theta;
% psi = TD.data_n580.psi;

figure(12); 
hb(1)=subplot(3,1,1);
plot(t,phi,'.');
xlim([0 max(t)])
grid on; 
xlabel('time (s)','FontSize',15,'Interpreter','latex'); 
ylabel('$\phi$ (deg) - n580','FontSize',15,'Interpreter','latex'); 
hb(2)=subplot(3,1,2);
plot(t,theta,'.');
xlim([0 max(t)])
grid on
xlabel('time (s)','FontSize',15,'Interpreter','latex'); 
ylabel('$\theta$ (deg) - n580','FontSize',15,'Interpreter','latex'); 
hb(3)=subplot(3,1,3);
plot(t,psi,'.'); 
xlim([0 max(t)])
ylim([-180 180])
grid on
xlabel('time (s)','FontSize',15,'Interpreter','latex'); 
ylabel('$\psi$ (deg) - n580','FontSize',15,'Interpreter','latex'); 
linkaxes(hb,'x')

%% Check Control Surfaces

delta_e = LP_15smooth( hampel( data_FMUR.delta_e ) ) ;
delta_a = LP_15smooth( hampel( data_FMUR.delta_a ) );
delta_a_r = LP_15smooth( hampel( data_FMUR.delta_a_r ) );
delta_a_l = LP_15smooth( hampel( data_FMUR.delta_a_l ) );
delta_r = LP_15smooth( hampel( data_FMUR.delta_r ) );
delta_f_r = LP_15smooth( hampel( data_FMUR.delta_f_r) );
delta_f_l = LP_15smooth( hampel( data_FMUR.delta_f_l ) );
delta_f = (delta_f_r + delta_f_l)/2;

% delta_e = Opt_LP(data_FMUR.delta_e,2,0.01,true);
% delta_a = Opt_LP(data_FMUR.delta_a,2,0.01,true);
% delta_a_r = Opt_LP(data_FMUR.delta_a_r,1,0.01,true);
% delta_a_l = Opt_LP(data_FMUR.delta_a_l,1,0.01,true);
% delta_r = Opt_LP(data_FMUR.delta_r,2,0.01,true);


% logic for flaps having the ultra noisy measurements from the EMI
for ii=1:length(delta_f)
    if delta_f(ii) > 7 && delta_f(ii) < 18
        delta_f(ii) = 15;
    elseif delta_f(ii) > 18
        delta_f(ii) = 30;
    else
        delta_f(ii) = 0;
    end
end
delta_f = hampel(delta_f,10);



figure(15);
title('Control Time history')
hc(1)=subplot(4,1,1);
plot(t, delta_e,'.r'); 
ylim([-30 30])
xlim([0 max(t)])
xlabel('time (s)','FontSize',20,'Interpreter','latex')
ylabel('$\delta_e$ (deg)','FontSize',20,'Interpreter','latex')
grid on
hc(2)=subplot(4,1,2);
plot(t, delta_a_r,'.r', t, delta_a_l,'.b', t, delta_a,'.g'); 
ylim([-30 30])
xlim([0 max(t)])
xlabel('time (s)','FontSize',20,'Interpreter','latex')
ylabel('$\delta_a$ (deg)','FontSize',20,'Interpreter','latex')
legend('right','left','avg')
grid on
hc(3)=subplot(4,1,3);
plot(t, delta_r,'.r'); 
ylim([-30 30])
xlim([0 max(t)])
xlabel('time (s)','FontSize',20,'Interpreter','latex')
ylabel('$\delta_r$ (deg)','FontSize',20,'Interpreter','latex')
grid on
hc(4)=subplot(4,1,4);
plot(t, delta_f,'.r'); 
ylim([0 30])
xlim([0 max(t)])
xlabel('time (s)','FontSize',20,'Interpreter','latex')
ylabel('$\delta_f$','FontSize',20,'Interpreter','latex')
grid on
linkaxes(hc,'x')

%% Check AoA & AoS

alpha = LP_15smooth( data_FMUR.alpha_CG);
beta = LP_15smooth( data_FMUR.beta_CG );

figure(14)
xlabel('$t$ $(s)$','FontSize',15,'Interpreter','latex')
yyaxis left
plot(t, alpha,'.')
ylim([-10 15])
ylabel(' $\alpha$(deg)','FontSize',15, 'Interpreter','latex')
yyaxis right
plot( t, beta,'.')
ylim([-20 20])
ylabel(' $\beta$ (deg)','FontSize',15, 'Interpreter','latex')
grid on
legend(' $\alpha$',' $\beta$','FontSize',15, 'Interpreter','latex','Location','southeast')

%% Check Mach Number
Mach = data_FMUR.Mach;

figure(105)
plot(t,Mach)
xlabel('time (s)','FontSize',20,'Interpreter','latex')
ylabel('$Mach$','FontSize',20,'Interpreter','latex')
grid on

%% Check Flight Path Angle
gamma = LP_15smooth( hampel( real( data_int.gamma.n580 ) ) );

figure(106)
plot(t,gamma,'.')
xlabel('time (s)','FontSize',20,'Interpreter','latex')
ylabel('$\gamma$ $(deg)$','FontSize',20,'Interpreter','latex')
grid on

%% Check Altitude

%altitude = LP_15smooth( data_FMUR.altitude.g_msl ); % from GPS
altitude = LP_15smooth( data_FMUR.altitude.p); % from static pressure

figure(107)
plot(t,altitude)
xlabel('time (s)','FontSize',20,'Interpreter','latex')
ylabel('$h$ $(ft)$','FontSize',20,'Interpreter','latex')
grid on

%% Check Load Factor

az =LP_15smooth( data_n580.az );

figure(108)
plot(t,az)
xlabel('time (s)','FontSize',20,'Interpreter','latex')
ylabel('$a_z$ $(ft/s^2)$','FontSize',20,'Interpreter','latex')
grid on

%% animation

N_start = 3.5*60*100; % index to start animation
N_end = 4.5*60*100; % index to stop animation
%N_start = 1;
%N_end = length(t);

% change path to C:\Users\Sam\ on desktop
% change path to C:\Users\jaege\ on laptop
model_info_file = 'C:\Users\Sam\MATLAB Drive\tools\3d_animations_MATLAB\3d_models\IBIS.mat';
frame_sample_time = 0.01;
speedx = 1; 
isave_movie = 1;
movie_file_name = 'IBIS_flight4_sturns_7_9_26.mp4';

% maximum control surface deflection
delta_a_max = 20; % [deg]
delta_e_max = 20; % [deg]

N_deflect = 2.0; % multiply deflection by this for visual

controls_deflection_deg = [N_deflect*delta_e(N_start:N_end), ...
            N_deflect*delta_a(N_start:N_end), ...
            N_deflect*delta_a(N_start:N_end), ...
            -N_deflect*delta_r(N_start:N_end), ...
            delta_f(N_start:N_end),...
            delta_f(N_start:N_end)]; % for IBIS model
%             delta_f_l(N_start:N_end),...
%             delta_f_r(N_start:N_end)]; % for IBIS model

aircraft_3d_animation(model_info_file,...
    psi(N_start:N_end), ...            Heading angle [deg]
    theta(N_start:N_end), ...              Pitch angle [deg]
    phi(N_start:N_end), ...               Roll angle [deg]
    delta_a(N_start:N_end)/(delta_a_max), ... Roll  stick command [-1,+1] [-1 -> left,            +1 -> right]
    delta_e(N_start:N_end)/(delta_e_max), ... Pitch stick command [-1,+1] [-1 -> full-back stick, +1 -> full-fwd stick]
    alpha(N_start:N_end), ...    AoA [deg]
    beta(N_start:N_end), ...  AoS [deg]
    gamma(N_start:N_end), ...   Flight path angle [deg]
    Mach(N_start:N_end), ...                   Mach number
    altitude(N_start:N_end)/3.28084, ...            Altitude [ft]
    -az(N_start:N_end)/32.174,  ...                  Vertical load factor [g]
    controls_deflection_deg, ...Flight control deflection (each column is a control surface)
    frame_sample_time, ...      Sample time [sec]
    speedx, ...                 Reproduction speed
    isave_movie, ...            Save the movie? 0-1
    movie_file_name);           % Movie file name