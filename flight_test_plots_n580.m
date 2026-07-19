% flight_test_plots_n580.fcn plots the raw n580 data from a data structure.
%
% flight_test_plots_n580(n580,tf_save_fig,i_n580_raw_se,i_n580_filt_se)
%
% INPUTS:
%   n580 - data structure
%
% OPTIONAL INPUTS:
%   tf_save_fig - true / false to save figures
%   i_n580_raw_se - 2x1 vector of starting and ending indicies asscoiated 
%       with "raw" data (IMU + attitude)
%   i_n580_filt_se - 2x1 vector of starting and ending indicies asscoiated 
%       with "filtered" data (velocity + position)
%
% OUTPUTS:
%   figure(101) - North-East-Down Position 3d plot
%   figure(1001) - position (lat/lon) zoomed to the ACRC field
%   figure(31) - angular velocity (body fixed x,y,z)
%   figure(32) - attitude (roll, pitch, yaw)
%   figure(33) - acceleration (body fixed x,y,z)
%   figure(37) - inertial velocity (North-East-Down)
%   figure(38) - altitude (MSL geometric)
%
% Sam Jaeger
% jaege246@umn.edu
% 7/13/2026


function flight_test_plots_n580(n580,varargin)
    dt= 0.01; % for n580 that UMN has

    N_raw =length(n580.raw(:,2));
    N_filt = length( n580.filt(:,2));

    narginchk(1,4)
    if nargin == 1
        id_s_raw = 1;
        id_e_raw = N_raw;

        id_s_filt = 1;
        id_e_filt = N_filt;

        tf_save_fig = false;
        
    elseif nargin == 2
        id_s_raw = 1;
        id_e_raw = N_raw;

        id_s_filt = 1;
        id_e_filt = N_filt;

        tf_save_fig = varargin{1};

    elseif nargin ==3
        error('Must supply raw and filtered starting and ending indices.')

    else
        tf_save_fig = varargin{1};
        i_n580_raw_se = varargin{2};
        i_n580_filt_se = varargin{3};
        id_s_raw = i_n580_raw_se(1);
        id_e_raw = i_n580_raw_se(2);
        id_s_filt = i_n580_filt_se(1);
        id_e_filt = i_n580_filt_se(2);
    end

    N_raw_plot = id_e_raw - id_s_raw;
    N_filt_plot = id_e_filt - id_s_filt;
    t_raw = 0:dt:((N_raw_plot)*dt);
    t_filt = 0:dt:((N_filt_plot)*dt);

    % ---------------------------------------------------------------------
    % position 3d (North-East-Down)
    figure(101)
    plot3(hampel(n580.filt(:,6))*3.28084,hampel(-n580.filt(:,5))*3.28084,hampel(n580.filt(:,9))*3.28084,'.')
    title('3D flight path','FontSize',20,'Interpreter','latex')
    xlabel('$X$ (ft)','FontSize',20,'Interpreter','latex')
    ylabel('$-Y$ (ft)','FontSize',20,'Interpreter','latex')
    zlabel('$-Z$ (ft)','FontSize',20,'Interpreter','latex')
    axis equal
    grid on

    % ---------------------------------------------------------------------
    % position (lat-lon)
    lat = n580.filt(id_s_filt:id_e_filt,7)*180/pi;
    lon = n580.filt(id_s_filt:id_e_filt,8)*180/pi;
    for ii=1:length(lat)
        if lat(ii) >= 90 || lat(ii) <= -90
            lat(ii) = 0;
        end
    end

    figure(1001)
    geoplot(lat,lon,'.r')
    geobasemap satellite
    title('n580 Position','FontSize',16,'Interpreter','latex')
    geolimits([45.325 45.3335],[-93.2363 -93.2257]) % ACRC Flying Field

    % ---------------------------------------------------------------------
    % angular velocity (body fixed)
    figure(31)
    hi(1)=subplot(3,1,1);
    plot(t_raw,(-n580.raw(id_s_raw:id_e_raw,10)*180/pi /dt),'.'); hold on
    plot(t_raw,LP_15smooth(-n580.raw(id_s_raw:id_e_raw,10)*180/pi /dt),'*')
    grid on
    xlabel('$t_{n580}$ $(s)$','Interpreter','latex','FontSize',15)
    ylabel('$p$ $(deg/s)$','Interpreter','latex','FontSize',15)
    title('Ang. Vel. Time History from n580 ','FontSize',16,'Interpreter','latex')

    hi(2)=subplot(3,1,2);
    plot(t_raw,(n580.raw(id_s_raw:id_e_raw,9)*180/pi /dt),'.'); hold on
    plot(t_raw,LP_15smooth(n580.raw(id_s_raw:id_e_raw,9)*180/pi /dt),'*')
    grid on
    xlabel('$t_{n580}$ $(s)$','Interpreter','latex','FontSize',15)
    ylabel('$q$ $(deg/s)$','Interpreter','latex','FontSize',15)

    hi(3)=subplot(3,1,3);
    plot(t_raw,(n580.raw(id_s_raw:id_e_raw,8)*180/pi /dt),'.'); hold on;
    plot(t_raw,LP_15smooth(n580.raw(id_s_raw:id_e_raw,8)*180/pi /dt),'*')
    grid on
    xlabel('$t_{n580}$ $(s)$','Interpreter','latex','FontSize',15)
    ylabel('$r$ $(deg/s)$','Interpreter','latex','FontSize',15)
    
    linkaxes(hi,'x')
    hold off

    % ---------------------------------------------------------------------
    % attitude
    figure(32); hold on
    hg(1)=subplot(3,1,1);
    plot(t_raw,-n580.raw(id_s_raw:id_e_raw,7)*180/pi,'.');
    grid on
    xlabel('$t_{n580}$ $(s)$','Interpreter','latex','FontSize',15)
    ylabel('Roll $(deg)$','Interpreter','latex','FontSize',15)
    title('Attitude Time History from n580 ','FontSize',16,'Interpreter','latex')

    hg(2)=subplot(3,1,2);
    plot(t_raw,-n580.raw(id_s_raw:id_e_raw,6)*180/pi,'.'); grid on
    xlabel('$t_{n580}$ $(s)$','Interpreter','latex','FontSize',15)
    ylabel('Pitch $(deg)$','Interpreter','latex','FontSize',15)

    hg(3)=subplot(3,1,3);
    plot(t_raw,-n580.raw(id_s_raw:id_e_raw,5)*180/pi+180,'.'); grid on
    ylim([0 360])
    xlabel('$t_{n580}$ $(s)$','Interpreter','latex','FontSize',15)
    ylabel('Yaw $(deg)$','Interpreter','latex','FontSize',15)

    linkaxes(hg,'x')
    hold off

    % ---------------------------------------------------------------------
    % acceleration
    figure(33); hold on
    hh(1)=subplot(3,1,1);
    plot(t_raw,(-n580.raw(id_s_raw:id_e_raw,13)*3.28084 /dt),'.');  hold on
    plot(t_raw,LP_15smooth(-n580.raw(id_s_raw:id_e_raw,13)*3.28084 /dt),'*')
    grid on
    xlabel('$t_{n580}$ $(s)$','Interpreter','latex','FontSize',15)
    ylabel('$a_x$ $(ft/s^2)$','Interpreter','latex','FontSize',15)
    title('Acceleration Time History from n580 ','FontSize',16,'Interpreter','latex')

    hh(2)=subplot(3,1,2);
    plot(t_raw,(n580.raw(id_s_raw:id_e_raw,12)*3.28084 /dt),'.'); hold on
    plot(t_raw,LP_15smooth(n580.raw(id_s_raw:id_e_raw,12)*3.28084 /dt),'*')
    grid on
    xlabel('$t_{n580}$ $(s)$','Interpreter','latex','FontSize',15)
    ylabel('$a_y$ $(ft/s^2)$','Interpreter','latex','FontSize',15)

    hh(3)=subplot(3,1,3);
    plot(t_raw,(n580.raw(id_s_raw:id_e_raw,11)*3.28084 /dt),'.'); hold on
    plot(t_raw,LP_15smooth(n580.raw(id_s_raw:id_e_raw,11)*3.28084 /dt),'*')
    grid on
    xlabel('$t_{n580}$ $(s)$','Interpreter','latex','FontSize',15)
    ylabel('$a_z$ $(ft/s^2)$','Interpreter','latex','FontSize',15)

    linkaxes(hh,'x')
    hold off


    % ---------------------------------------------------------------------
    % velocity
    figure(37); hold on
    plot(t_filt,n580.filt(id_s_filt:id_e_filt,13)*3.28084,'.'); 
    plot(t_filt,n580.filt(id_s_filt:id_e_filt,14)*3.28084,'.'); 
    plot(t_filt,n580.filt(id_s_filt:id_e_filt,15)*3.28084,'.'); 
    grid on
    xlabel('$t_{n580}$ $(s)$','Interpreter','latex','FontSize',15)
    ylabel('V $(ft/s)$','Interpreter','latex','FontSize',15)
    title('Inertial Velocity Time History from n580 ','FontSize',16,'Interpreter','latex')
    legend('North','East','Down','location','northwest','Interpreter','latex')
    hold off

    % ---------------------------------------------------------------------
    % altitude 
    figure(38)
    plot(t_filt,n580.filt(:,9)*3.28084,'.'); hold on
    ylim([600 max(n580.filt(:,9)*3.28084)*1.2])
    grid on
    xlabel('$t_{n580}$ $(s)$','Interpreter','latex','FontSize',15)
    ylabel('Altitude MSL $(ft)$','Interpreter','latex','FontSize',15)
    title('Altitude MSL Time History from n580','FontSize',16,'Interpreter','latex')
    hold off

    % ---------------------------------------------------------------------
    if tf_save_fig == true
        fig_vec = [101, 1001, 31, 32, 33, 37, 38];
        fig_label = {'NED_Position_3d';
                     'Lat_Lon_Position';
                     'Angular_Velocity';
                     'Attitude';
                     'Acceleration';
                     'Inertial_Velocity',
                     'Altitude'};
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
                fname = string(append(fig_label(ii),'_n580_',flight_date)); 
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