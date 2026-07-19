% flight_test_plots_FMUR.fcn plots only the FMUR data from a provided data 
% structure and calibration file. Figures are numbered the same as in 
% flight_test_plots.fcn
%
% flight_test_plots_FMUR(test_data.FMUR,cal,tf_save_fig,i_FMUR_s,i_FMUR_e)
%
% INPUTS:
%   FMUR - data structure
%   cal - calibration data structure
%
% OPTIONAL INPUT:
%   tf_save_fig - true / false to save figures
%   i_FMUR_s - index to start the FMUR data plotting
%   i_FMUR_e - index to end the FMUR data plotting
%
% OUTPUTS: generates the following figures 
%   figure(101) - North-East-Down Position 3d plot
%   figure(1001) - position (lat-lon) 
%   figure(10) - velocity (calibrated airspeed and total inertial)
%   figure(11) - angular velocity (body fixed)
%   figure(12) - attitude (roll, pitch, yaw)
%   figure(13) - acceleration (body fixed)
%   figure(14) - aero angle (alpha, beta)
%   figure(15) - control surface deflections
%   figure(16) - flap deflection
%   figure(17) - inertial velocity (North-East-Down)
%   figure(18) - altitude (geometric and pressure)
%   figure(19) - differential pressure
%   figure(20) - alpha / q / delta_e
%   figure(21) - beta / p / delta_a
%   figure(22) - beta / r / delta_r
%   figure(23) - rpm / a_x / V_c
%
% Sam Jaeger
% jaege246@umn.edu
% 7/13/2026

function flight_test_plots_FMUR(FMUR,cal,varargin)

    % logic for optional inputs
    narginchk(2,5)
    if nargin == 2
        i_FMUR_s = 1;
        i_FMUR_e = length(FMUR.sys.time_s);
        tf_save_fig = false;
    elseif nargin == 3
        tf_save_fig = varargin{1};
        i_FMUR_s = 1;
        i_FMUR_e = length(FMUR.sys.time_s);
    elseif nargin == 4
        disp('Invalid number of input arguments.')
        error('Must supply a starting and ending index as two separate arguments in addition to save figure logic.')
    elseif nargin == 5
        tf_save_fig = varargin{1};
        i_FMUR_s = varargin{2};
        i_FMUR_e = varargin{3};
    end

    % use FMUR system time for everything
    t = FMUR.sys.time_s(i_FMUR_s:i_FMUR_e);

    % alpha (angle of attack deg)
    alpha = polyval(cal.p_alpha,FMUR.adc_volt((i_FMUR_s:i_FMUR_e),2));
    
    % beta / beta_f (flank angle deg)
    beta = polyval(cal.p_beta_f,FMUR.adc_volt((i_FMUR_s:i_FMUR_e),1));
    %beta_f = atan2d( tand(beta),cosd(alpha));

    D_p = FMUR.pres.diff.pres_pa(i_FMUR_s:i_FMUR_e)*0.020885434273039;
    V_c = real(calibrated_airspeed(D_p,cal.lam_Dp,cal.b_Dp));
  
    % ---------------------------------------------------------------------
    % North-East-Down 3d position
    figure(101)
    plot3(FMUR.nav.ned.pos_m(i_FMUR_s:i_FMUR_e,1)*3.28084,-FMUR.nav.ned.pos_m(i_FMUR_s:i_FMUR_e,2)*3.28084,FMUR.nav.ned.pos_m(i_FMUR_s:i_FMUR_e,3)*3.28084,'.')
    title('3D flight path','FontSize',15,'Interpreter','latex')
    xlabel('$X$ (ft)','FontSize',15,'Interpreter','latex')
    ylabel('$-Y$ (ft)','FontSize',15,'Interpreter','latex')
    zlabel('$-Z$ (ft)','FontSize',15,'Interpreter','latex')
    axis equal
    grid on

    % ---------------------------------------------------------------------
    % lat-lon position
    figure(1001)
    geoplot(FMUR.nav.lat_rad(i_FMUR_s:i_FMUR_e)*180/pi,FMUR.nav.lon_rad(i_FMUR_s:i_FMUR_e)*180/pi,'.r')
    geobasemap satellite
    title('FMU-R Position','FontSize',16,'Interpreter','latex')
    geolimits([45.325 45.3335],[-93.2363 -93.2257]) % ACRC Flying Field

    % ---------------------------------------------------------------------
    % Velocity - airspeed and total inertial
    figure(10)
    plot(t,V_c,'.'); hold on;
    plot(t,vecnorm(FMUR.gnss.ned_vel_mps(i_FMUR_s:i_FMUR_e,:),2,2)*3.28084,'.')
    grid on
    xlabel('$t_{FMUR}$ $(s)$','Interpreter','latex','FontSize',15)
    ylabel('$V$ $(ft/s)$','Interpreter','latex','FontSize',15)
    legend('Calibrated Airspeed','Total Inertial Velocity')

    % ---------------------------------------------------------------------
    % angular velocity
    figure(11)
    hf(1)=subplot(3,1,1);
    plot(t,-FMUR.imu.gyro_radps(i_FMUR_s:i_FMUR_e,1)*180/pi,'.'); hold on
    plot(t,LP_15smooth(-FMUR.imu.gyro_radps(i_FMUR_s:i_FMUR_e,1)*180/pi),'*');
    grid on
    xlabel('$t_{FMUR}$ $(s)$','Interpreter','latex','FontSize',15)
    ylabel('$p$ $(deg/s)$','Interpreter','latex','FontSize',15)
    title('FMU-R Gyro Time History ','Raw and Filtered','FontSize',16,'Interpreter','latex')

    hf(2)=subplot(3,1,2);
    plot(t,-FMUR.imu.gyro_radps(i_FMUR_s:i_FMUR_e,2)*180/pi,'.'); hold on
    plot(t,LP_15smooth(-FMUR.imu.gyro_radps(i_FMUR_s:i_FMUR_e,2)*180/pi),'*');
    grid on
    xlabel('$t_{FMUR}$ $(s)$','Interpreter','latex','FontSize',15)
    ylabel('$q$ $(deg/s)$','Interpreter','latex','FontSize',15)

    hf(3)=subplot(3,1,3);
    plot(t,-FMUR.imu.gyro_radps(i_FMUR_s:i_FMUR_e,3)*180/pi,'.'); hold on
    plot(t,LP_15smooth(-FMUR.imu.gyro_radps(i_FMUR_s:i_FMUR_e,3)*180/pi),'*');
    grid on
    xlabel('$t_{FMUR}$ $(s)$','Interpreter','latex','FontSize',15)
    ylabel('$r$ $deg/s)$','Interpreter','latex','FontSize',15)

    linkaxes(hf,'x')
    hold off

    % ---------------------------------------------------------------------
    % attitude (roll, pitch, yaw)
    figure(12)
    hi(1)=subplot(3,1,1);
    plot(t,FMUR.nav.roll_rad(i_FMUR_s:i_FMUR_e)*180/pi,'.'); 
    grid on; 
    xlabel('time (s)','FontSize',15,'Interpreter','latex'); 
    ylabel('$\phi$ (deg) - n580','FontSize',15,'Interpreter','latex'); 
    hi(2)=subplot(3,1,2);
    plot(t,FMUR.nav.pitch_rad(i_FMUR_s:i_FMUR_e)*180/pi,'.');
    grid on
    xlabel('time (s)','FontSize',15,'Interpreter','latex'); 
    ylabel('$\theta$ (deg) - n580','FontSize',15,'Interpreter','latex'); 
    hi(3)=subplot(3,1,3);
    plot(t,FMUR.nav.heading_rad(i_FMUR_s:i_FMUR_e)*180/pi,'.'); 
    grid on
    xlabel('time (s)','FontSize',15,'Interpreter','latex'); 
    ylabel('$\psi$ (deg) - n580','FontSize',15,'Interpreter','latex'); 
    linkaxes(hi,'x')

    % ---------------------------------------------------------------------
    % acceleration
    figure(13)
    he(1)=subplot(3,1,1);
    plot(t,-FMUR.imu.accel_mps2(i_FMUR_s:i_FMUR_e,1)*3.28084,'.'); hold on
    plot(t,LP_15smooth(-FMUR.imu.accel_mps2(i_FMUR_s:i_FMUR_e,1)*3.28084),'*');
    grid on
    xlabel('$t_{FMUR}$ $(s)$','Interpreter','latex','FontSize',15)
    ylabel('$a_{x}$ $(ft/s^2)$','Interpreter','latex','FontSize',15)
    title('FMU-R Accel Time History ','Raw and Filtered','FontSize',16,'Interpreter','latex')

    he(2)=subplot(3,1,2);
    plot(t,-FMUR.imu.accel_mps2(i_FMUR_s:i_FMUR_e,2)*3.28084,'.'); hold on
    plot(t,LP_15smooth(-FMUR.imu.accel_mps2(i_FMUR_s:i_FMUR_e,2)*3.28084),'*');
    grid on
    xlabel('$t_{FMUR}$ $(s)$','Interpreter','latex','FontSize',15)
    ylabel('$a_{y}$ $(ft/s^2)$','Interpreter','latex','FontSize',15)

    he(3)=subplot(3,1,3);
    plot(t,-FMUR.imu.accel_mps2(i_FMUR_s:i_FMUR_e,3)*3.28084,'.'); hold on
    plot(t,LP_15smooth(-FMUR.imu.accel_mps2(i_FMUR_s:i_FMUR_e,3)*3.28084),'*');
    grid on
    xlabel('$t_{FMUR}$ $(s)$','Interpreter','latex','FontSize',15)
    ylabel('$a_{z}$ $(ft/s^2)$','Interpreter','latex','FontSize',15)

    linkaxes(he,'x')
    hold off

    % ---------------------------------------------------------------------
    % aero angle (alpha, beta)
    figure(14)
    xlabel('$t_{FMUR}$ $(s)$','FontSize',15,'Interpreter','latex')
    yyaxis left
    plot(t, alpha,'.')
    ylabel(' $\alpha$(deg)','FontSize',15, 'Interpreter','latex')
    yyaxis right
    plot( t, beta,'.')
    ylabel(' $\beta$ (deg)','FontSize',15, 'Interpreter','latex')
    grid on
    legend(' $\alpha$',' $\beta$','FontSize',15, 'Interpreter','latex','Location','southeast')

    % ---------------------------------------------------------------------
    % control surface time histories

    % delta_e
    delta_e = polyval(cal.p_delta_e,FMUR.adc_volt(i_FMUR_s:i_FMUR_e,3));

    % delta_a
    delta_a_r = polyval(cal.p_delta_a_r,FMUR.adc_volt(i_FMUR_s:i_FMUR_e,5));
    delta_a_l = polyval(cal.p_delta_a_l,FMUR.adc_volt(i_FMUR_s:i_FMUR_e,6));
    delta_a = (delta_a_r + delta_a_l)/2;

    % delta_r
    delta_r = polyval(cal.p_delta_r, FMUR.adc_volt(i_FMUR_s:i_FMUR_e,4));

    % delta_f
    delta_f_r = polyval(cal.p_delta_f_r,FMUR.adc_volt(i_FMUR_s:i_FMUR_e,8));
    delta_f_l = polyval(cal.p_delta_f_l,FMUR.adc_volt(i_FMUR_s:i_FMUR_e,7));
    delta_f = (delta_f_r + delta_f_l)/2;


    figure(15);
    title('Control Time history','RPM Hampel Filtered')
    hc(1)=subplot(4,1,1);
    plot(t, hampel(FMUR.pwr.rpm(i_FMUR_s:i_FMUR_e)),'.r'); ylim([0 14000])
    xlabel('$t_{FMUR}$ $(s)$','FontSize',15,'Interpreter','latex')
    ylabel('$rpm$ ','FontSize',15,'Interpreter','latex')
    grid on
    hc(2)=subplot(4,1,2);
    plot(t, delta_e,'.r'); ylim([-30 30])
    xlabel('$t_{FMUR}$ $(s)$','FontSize',15,'Interpreter','latex')
    ylabel('$\delta_e$ (deg)','FontSize',15,'Interpreter','latex')
    grid on
    hc(3)=subplot(4,1,3);
    plot(t, delta_a_r,'.r', t, delta_a_l,'.b', t, delta_a,'.g'); ylim([-30 30])
    xlabel('$t_{FMUR}$ $(s)$','FontSize',15,'Interpreter','latex')
    ylabel('$\delta_a$ (deg)','FontSize',15,'Interpreter','latex')
    legend('right','left','avg')
    grid on
    hc(4)=subplot(4,1,4);
    plot(t, delta_r,'.r'); ylim([-30 30])
    xlabel('$t_{FMUR}$ $(s)$','FontSize',15,'Interpreter','latex')
    ylabel('$\delta_r$ (deg)','FontSize',15,'Interpreter','latex')
    grid on
    linkaxes(hc,'x')

    %----------------------------------------------------------------------
    % flap deflection
    figure(16),plot(t,delta_f,'.'); 
    grid on
    xlabel('$t_{FMUR}$ $(s)$','Interpreter','latex','FontSize',15)
    ylabel('$\delta_f$ $(deg)$','Interpreter','latex','FontSize',15)
    title('Flap Time History from pot. ','FontSize',16,'Interpreter','latex')

    % ---------------------------------------------------------------------
    % North-East-Down velocity
    figure(17); hold on
    plot(t,FMUR.nav.ned.vel_mps(i_FMUR_s:i_FMUR_e,1)*3.28084,'.'); 
    plot(t,FMUR.nav.ned.vel_mps(i_FMUR_s:i_FMUR_e,2)*3.28084,'.'); 
    plot(t,FMUR.nav.ned.vel_mps(i_FMUR_s:i_FMUR_e,3)*3.28084,'.'); 
    grid on
    xlabel('$t_{FMUR}$ $(s)$','Interpreter','latex','FontSize',15)
    ylabel('V $(ft/s)$','Interpreter','latex','FontSize',15)
    title('Inertial Velocity Time History from FMUR ','FontSize',16,'Interpreter','latex')
    legend('North','East','Down','location','northwest','Interpreter','latex')
    hold off

    % ---------------------------------------------------------------------
    % altitude 
    % pressure ratio
    [~,~,p_SL,~,~] = ATMOS(0,'SI');
    delta_rat = FMUR.pres.static.pres_pa(i_FMUR_s:i_FMUR_e)/p_SL - cal.pres_stat_bias/47.880258888889/p_SL;
    h_p = (1 - (delta_rat.^(1/5.2559)))/(6.87559*10^(-6)); % pressure altitude

    figure(18)
    plot(t,FMUR.gnss.alt_msl_m(i_FMUR_s:i_FMUR_e)*3.28084,'.'); hold on
    plot(t,h_p,'.');
    ylim([600 1500])
    %ylim([600 max(FMUR.gnss.alt_msl_m(i_FMUR_s:i_FMUR_e)*3.28084)*1.2])
    grid on
    legend('GNSS altitude (MSL)','pressure altitude','Interpreter','latex')
    xlabel('$t_{FMUR}$ $(s)$','Interpreter','latex','FontSize',15)
    ylabel('Altitude MSL $(ft)$','Interpreter','latex','FontSize',15)
    title('Altitude MSL Time History from FMUR GPS','FontSize',16,'Interpreter','latex')
    hold off

    % ---------------------------------------------------------------------
    % differential pressure
    figure(19)
    plot(t,FMUR.pres.diff.pres_pa(i_FMUR_s:i_FMUR_e)*0.020885434273039,'.'); hold on;
    grid on
    xlabel('$t_{FMUR}$ $(s)$','Interpreter','latex','FontSize',15)
    ylabel('$\Delta p$ $(lb/ft^2)$','Interpreter','latex','FontSize',15)

    % ---------------------------------------------------------------------
    % alpha / q / delta_e
    figure(20)
    ha(1)=subplot(3,1,1);
    plot(t,alpha,'.'); 
    grid on
    xlabel('$t_{FMUR}$ $(s)$','Interpreter','latex','FontSize',15)
    ylabel('$\alpha$ $(deg)$','Interpreter','latex','FontSize',15)
    title('$\alpha$ / q / elevator Time History from pot. ','FontSize',16,'Interpreter','latex')

    ha(2)=subplot(3,1,2);
    plot(t,-FMUR.imu.gyro_radps(i_FMUR_s:i_FMUR_e,2)*180/pi,'.'); 
    grid on
    xlabel('$t_{FMUR}$ $(s)$','Interpreter','latex','FontSize',15)
    ylabel('$q$ $(deg/s)$','Interpreter','latex','FontSize',15)

    ha(3)=subplot(3,1,3);
    plot(t,delta_e,'.'); hold on;
    grid on
    xlabel('$t_{FMUR}$ $(s)$','Interpreter','latex','FontSize',15)
    ylabel('$\delta_e$ $(deg)$','Interpreter','latex','FontSize',15)
    
    linkaxes(ha,'x')
    hold off

    % ---------------------------------------------------------------------
    % beta / p / delta_a
    figure(21)
    hb(1)=subplot(3,1,1);
    plot(t,beta,'.'); 
    grid on
    xlabel('$t_{FMUR}$ $(s)$','Interpreter','latex','FontSize',15)
    ylabel('$\beta$ $(deg)$','Interpreter','latex','FontSize',15)
    title('$\beta$ / p / aileron Time History from pot. ','FontSize',16,'Interpreter','latex')

    hb(2)=subplot(3,1,2);
    plot(t,-FMUR.imu.gyro_radps(i_FMUR_s:i_FMUR_e,1)*180/pi,'.'); 
    grid on
    xlabel('$t_{FMUR}$ $(s)$','Interpreter','latex','FontSize',15)
    ylabel('$p$ $(deg/s)$','Interpreter','latex','FontSize',15)

    hb(3)=subplot(3,1,3);
    plot(t,delta_a,'.'); hold on;
    grid on
    xlabel('$t_{FMUR}$ $(s)$','Interpreter','latex','FontSize',15)
    ylabel('$\delta_a$ $(deg)$','Interpreter','latex','FontSize',15)
    
    linkaxes(hb,'x')
    hold off

    % ---------------------------------------------------------------------
    % beta / r / delta_r
    figure(22)
    hj(1)=subplot(3,1,1);
    plot(t,beta,'.'); 
    grid on
    xlabel('$t_{FMUR}$ $(s)$','Interpreter','latex','FontSize',15)
    ylabel('$\beta$ $(deg)$','Interpreter','latex','FontSize',15)
    title('$\beta$ / r / rudder Time History from pot. ','FontSize',16,'Interpreter','latex')

    hj(2)=subplot(3,1,2);
    plot(t,-FMUR.imu.gyro_radps(i_FMUR_s:i_FMUR_e,3)*180/pi,'.'); 
    grid on
    xlabel('$t_{FMUR}$ $(s)$','Interpreter','latex','FontSize',15)
    ylabel('$r$ $(deg/s)$','Interpreter','latex','FontSize',15)

    hj(3)=subplot(3,1,3);
    plot(t,delta_r,'.'); hold on;
    grid on
    xlabel('$t_{FMUR}$ $(s)$','Interpreter','latex','FontSize',15)
    ylabel('$\delta_r$ $(deg)$','Interpreter','latex','FontSize',15)
    
    linkaxes(hj,'x')
    hold off

    % ---------------------------------------------------------------------
    % rpm / ax / V_c
    figure(23)
    hd(1)=subplot(3,1,1);
    plot(t,hampel(FMUR.pwr.rpm(i_FMUR_s:i_FMUR_e)),'.'); 
    ylim([0 15000])
    grid on
    xlabel('$t_{FMUR}$ $(s)$','Interpreter','latex','FontSize',15)
    ylabel('$rpm$','Interpreter','latex','FontSize',15)
    title('rpm / $a_x$ / $\Delta p$ Time History ','RPM Hampel Filtered, Accel Low Pass Filtered','FontSize',16,'Interpreter','latex')

    hd(2)=subplot(3,1,2);
    plot(t,-FMUR.imu.accel_mps2(i_FMUR_s:i_FMUR_e,1)*3.28084,'.'); hold on
    plot(t,LP_15smooth(-FMUR.imu.accel_mps2(i_FMUR_s:i_FMUR_e,1)*3.28084),'*');
    grid on
    xlabel('$t_{FMUR}$ $(s)$','Interpreter','latex','FontSize',15)
    ylabel('$a_{x}$ $(ft/s^2)$','Interpreter','latex','FontSize',15)

    hd(3)=subplot(3,1,3);
    plot(t,V_c,'.'); hold on;
    grid on
    xlabel('$t_{FMUR}$ $(s)$','Interpreter','latex','FontSize',15)
    ylabel('$V_{calibrated}$ $(ft/s)$','Interpreter','latex','FontSize',15)

    linkaxes(hd,'x')
    hold off

    % ---------------------------------------------------------------------
    if tf_save_fig == true
        fig_vec = [101, 1001, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23];
        fig_label = {'NED_Position_3d';
                     'Lat_Lon_Position';
                     'Cal_Airspeed_Velocity';
                     'Angular_Velocity';
                     'Attitude';
                     'Acceleration';
                     'Alpha_Beta';
                     'Control_Deflection';
                     'Flap_Deflection';
                     'Inertial_Velocity';
                     'Altitude';
                     'Differential_Pressure'
                     'Alpha_q_Elevator';
                     'Beta_p_Aileron';
                     'Beta_r_Rudder';
                     'RPM_ax_Cal_Airspeed'};
        warning('off','all');
        disp('-------------------------------------')
        disp('-------- SAVE FIGURE ROUTINE --------')
        disp('-------------------------------------')
        disp('Input flight test date and number in the following format:')
        disp( '  MM_DD_YYYY-X')
        flight_date = input('->',"s");
        
        for ii=1:length(fig_vec)
            figure(fig_vec(ii))
            disp('----------------------------')
            disp(append(string(fig_label(ii)),' Plot'))
            disp('Zoom plot and hit enter to continue.')
            pause
            disp('Save current figure?')
            save_i_fig = input('true / false: ');
            if save_i_fig == true
                disp('saving figure...')
                fname = string(append(fig_label(ii),'_FMUR_',flight_date)); 
                if fig_vec(ii) == 1001 || fig_vec(ii) == 101
                    saveas(gcf,fname,'png')
                else
                    saveas(gcf,fname,'svg')
                    title('')
                    saveas(gcf,fname,'epsc')
                    saveas(gcf,fname,'png')
                end
            end
            close(fig_vec(ii))
        end
    end
end