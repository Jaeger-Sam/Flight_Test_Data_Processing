% crop_flight_test_data.fcn finds the time indicies from the test_data
% structure from read_flight_test_data.fcn.
%
% [t_GNSS_se,i_FMUR_se, i_n580_raw_se,i_n580_filt_se,maneuver_label] = crop_flight_test_data(test_data)
%
% INPUTS:
%   test_data: data structure from read_flight_test_data.fcn
%
% OPTIONAL INPUTS:
%   crop_data_man_tf: true / false to crop data manually. Otherwise will
%       ask for user input.
%
% OUTPUTS:
%   t_GNSS_se: matrix of GNSS times for the cropping data. 
%       The first column corresponds to the starting time, the second 
%       column corresponds to the ending time. The rows correspond to a 
%       particular maneuver.
%   i_FMUR_se: matrix of cropping indicies for the FMUR data. 
%       The first column corresponds to the starting index, the second 
%       column corresponds to the ending index. The rows correspond to a 
%       particular maneuver.
%   i_n580_raw_se: matrix of cropping indicies for the raw n580 data. 
%       The first column corresponds to the starting index, the second 
%       column corresponds to the ending index. The rows correspond to a 
%       particular maneuver.
%   i_n580_filt_se: matrix of cropping indicies for the filtered n580 data. 
%       The first column corresponds to the starting index, the second 
%       column corresponds to the ending index. The rows correspond to a 
%       particular maneuver.  
%   maneuver_label: string vector of maneuver labels.
%
% Sam Jaeger
% jaege246@umn.edu
% 7/1/2026


function [t_GNSS_se,i_FMUR_se, i_n580_raw_se,i_n580_filt_se,maneuver_label] = crop_flight_test_data(test_data,varargin)

    narginchk(1,2)
    if nargin ==2
        crop_data_man_tf = varargin{1};
    else
        disp('Crop data manually?')
        crop_data_man_tf = input('true/false:  ');
    end
    
    if crop_data_man_tf == true
        disp('==========================================')
        disp('MANUAL CROPPING MODE =====================')

        % initalize data structures
        t_GNSS_se = [];
        i_FMUR_se = [];
        i_n580_raw_se = [];
        i_n580_filt_se = [];

        add_maneuver = true;
        for jj=1:10 % loop over data cropping 
        
        if jj>1
            disp('Add maneuver?')
            add_maneuver = input('true / false:   ');
        end

        if add_maneuver == false
            break
        end
        disp('Which variable to look at to use to crop?')
        disp('     altitude_MSL ================ 0')
        disp('     Inertial_Velocity =========== 1')
        disp('     roll / pitch / yaw (n580) === 2')
        disp('     a_{xyz} (n580 filt.) ======== 3')
        disp('     omega_{xyz} (n580 filt.) ==== 4')
        disp('     rpm ========================= 5')
        disp('     delta_e / delta_a / delta_r = 6')
        disp('     delta_f ===================== 7')
        disp('     DELTA_p / p_static ========== 8')
        disp('     north / east position (n580)= 9')
        disp('     alpha / q / delta_e ========= 10')
        disp('     beta / p / delta_a ========== 11')
        disp('     beta / r / delta_r ========== 12')
        disp('     rpm / a_x / Delta_p ========= 13')
        crop_var = input('Input var #:                       ');
        
        if crop_var==0 % Alitude MSL
            figure(1),plot(test_data.FMUR.gnss.alt_msl_m*3.28084,'.')
            grid on
            xlabel('timestep, $k$','Interpreter','latex','FontSize',15)
            ylabel('Altitude MSL $(ft)$','Interpreter','latex','FontSize',15)
            title('Altitude MSL Time History from FMUR GPS','FontSize',16,'Interpreter','latex')
            subtitle('select 2 points to crop data','FontSize',15,'Interpreter','latex')

            disp('zoom plot to desired section of data and hit enter.')
            pause

            disp('Select two sequential data points:')
            [ind,~]=ginput(2);
            i_FMUR_s = round(ind(1));
            i_FMUR_e = round(ind(2));
            if i_FMUR_e < i_FMUR_s
                error('Datapoints must be sequential')
            end
            close(1)

            t_GNSS_start = test_data.FMUR.sys.time_gnss_s(i_FMUR_s);
            t_GNSS_end = test_data.FMUR.sys.time_gnss_s(i_FMUR_e);

            [~,id_s_raw] = min(abs(test_data.n580.raw(:,2) - t_GNSS_start));
            [~,id_e_raw] = min(abs(test_data.n580.raw(:,2) - t_GNSS_end));

            [~,id_s_filt] = min(abs(test_data.n580.filt(:,2) - t_GNSS_start));
            [~,id_e_filt] = min(abs(test_data.n580.filt(:,2) - t_GNSS_end));

        elseif crop_var==1 % Inertial Velocity
            figure(1),plot(vecnorm(test_data.n580.filt(:,13:15),2,2)*3.28084,'.')
            grid on
            xlabel('timestep, $k$','Interpreter','latex','FontSize',15)
            ylabel('Inerital Velocity $(ft/s)$','Interpreter','latex','FontSize',15)
            title('Inertial Velocity Time History from n580 ','FontSize',16,'Interpreter','latex')
            subtitle('select 2 points to crop data','FontSize',15,'Interpreter','latex')

            disp('zoom plot to desired section of data and hit enter.')
            pause

            disp('Select two sequential data points:')
            [ind,~]=ginput(2);
            id_s_filt = round(ind(1));
            id_e_filt = round(ind(2));
            if id_e_filt < id_s_filt
                error('Datapoints must be sequential')
            end
            close(1)

            t_GNSS_start = test_data.n580.filt(id_s_filt,2);
            t_GNSS_end = test_data.n580.filt(id_e_filt,2);

            [~,id_s_raw] = min(abs(test_data.n580.raw(:,2) - t_GNSS_start));
            [~,id_e_raw] = min(abs(test_data.n580.raw(:,2) - t_GNSS_end));

            [~,i_FMUR_s] = min(abs(test_data.FMUR.sys.time_gnss_s - t_GNSS_start));
            [~,i_FMUR_e] = min(abs(test_data.FMUR.sys.time_gnss_s - t_GNSS_end));

        elseif crop_var==2 % roll / pitch / yaw from n580
            figure(1); hold on
            ha(1)=subplot(3,1,1);
            plot(-test_data.n580.raw(:,7)*180/pi,'.');
            grid on
            xlabel('timestep, $k$','Interpreter','latex','FontSize',15)
            ylabel('Roll $(deg)$','Interpreter','latex','FontSize',15)
            title('Attitude Time History from n580 ','FontSize',16,'Interpreter','latex')
            subtitle('select 2 points to crop data','FontSize',15,'Interpreter','latex')

            ha(2)=subplot(3,1,2);
            plot(-test_data.n580.raw(:,6)*180/pi,'.'); grid on
            xlabel('timestep, $k$','Interpreter','latex','FontSize',15)
            ylabel('Pitch $(deg)$','Interpreter','latex','FontSize',15)

            ha(3)=subplot(3,1,3);
            plot(-test_data.n580.raw(:,5)*180/pi+180,'.'); grid on
            xlabel('timestep, $k$','Interpreter','latex','FontSize',15)
            ylabel('Yaw $(deg)$','Interpreter','latex','FontSize',15)
    
            linkaxes(ha,'x')
            hold off

            disp('zoom plot to desired section of data and hit enter.')
            pause

            disp('Select two sequential data points:')
            [ind,~]=ginput(2);
            id_s_raw = round(ind(1));
            id_e_raw = round(ind(2));
            if id_e_raw < id_s_raw
                error('Datapoints must be sequential')
            end
            close(1)

            t_GNSS_start = test_data.n580.raw(id_s_raw,2);
            t_GNSS_end = test_data.n580.raw(id_e_raw,2);

            [~,id_s_filt] = min(abs(test_data.n580.filt(:,2) - t_GNSS_start));
            [~,id_e_filt] = min(abs(test_data.n580.filt(:,2) - t_GNSS_end));

            [~,i_FMUR_s] = min(abs(test_data.FMUR.sys.time_gnss_s - t_GNSS_start));
            [~,i_FMUR_e] = min(abs(test_data.FMUR.sys.time_gnss_s - t_GNSS_end));

        elseif crop_var==3 % a_{xyz} (n580 filt.)
            dt= 0.01; % for n580 that UMN has
            figure(1); hold on
            ha(1)=subplot(3,1,1);
            plot((-test_data.n580.raw(:,13)*3.28084 /dt),'.');  hold on
            plot(LP_15smooth(-test_data.n580.raw(:,13)*3.28084 /dt),'*')
            grid on
            xlabel('timestep, $k$','Interpreter','latex','FontSize',15)
            ylabel('$a_x$ $(ft/s^2)$','Interpreter','latex','FontSize',15)
            title('Acceleration Time History from n580 ','FontSize',16,'Interpreter','latex')
            subtitle('select 2 points to crop data','FontSize',15,'Interpreter','latex')

            ha(2)=subplot(3,1,2);
            plot((test_data.n580.raw(:,12)*3.28084 /dt),'.'); hold on
            plot(LP_15smooth(test_data.n580.raw(:,12)*3.28084 /dt),'*')
            grid on
            xlabel('timestep, $k$','Interpreter','latex','FontSize',15)
            ylabel('$a_y$ $(ft/s^2)$','Interpreter','latex','FontSize',15)

            ha(3)=subplot(3,1,3);
            plot((test_data.n580.raw(:,11)*3.28084 /dt),'.'); hold on
            plot(LP_15smooth(test_data.n580.raw(:,11)*3.28084 /dt),'*')
            grid on
            xlabel('timestep, $k$','Interpreter','latex','FontSize',15)
            ylabel('$a_z$ $(ft/s^2)$','Interpreter','latex','FontSize',15)

            linkaxes(ha,'x')
            hold off

            disp('zoom plot to desired section of data and hit enter.')
            pause

            disp('Select two sequential data points:')
            [ind,~]=ginput(2);
            id_s_raw = round(ind(1));
            id_e_raw = round(ind(2));
            if id_e_raw < id_s_raw
                error('Datapoints must be sequential')
            end
            close(1)

            t_GNSS_start = test_data.n580.raw(id_s_raw,2);
            t_GNSS_end = test_data.n580.raw(id_e_raw,2);

            [~,id_s_filt] = min(abs(test_data.n580.filt(:,2) - t_GNSS_start));
            [~,id_e_filt] = min(abs(test_data.n580.filt(:,2) - t_GNSS_end));

            [~,i_FMUR_s] = min(abs(test_data.FMUR.sys.time_gnss_s - t_GNSS_start));
            [~,i_FMUR_e] = min(abs(test_data.FMUR.sys.time_gnss_s - t_GNSS_end));

        elseif crop_var==4 % omega_{xyz} (n580 filt.)
            dt= 0.01; % for n580 that UMN has
            figure(1)
            ha(1)=subplot(3,1,1);
            plot((-test_data.n580.raw(:,10)*180/pi /dt),'.'); hold on
            plot(LP_15smooth(-test_data.n580.raw(:,10)*180/pi /dt),'*')
            grid on
            xlabel('timestep, $k$','Interpreter','latex','FontSize',15)
            ylabel('$p$ $(deg/s)$','Interpreter','latex','FontSize',15)
            title('Ang. Vel. Time History from n580 ','FontSize',16,'Interpreter','latex')
            subtitle('select 2 points to crop data','FontSize',15,'Interpreter','latex')

            ha(2)=subplot(3,1,2);
            plot((test_data.n580.raw(:,9)*180/pi /dt),'.'); hold on
            plot(LP_15smooth(test_data.n580.raw(:,9)*180/pi /dt),'*')
            grid on
            xlabel('timestep, $k$','Interpreter','latex','FontSize',15)
            ylabel('$q$ $(deg/s)$','Interpreter','latex','FontSize',15)

            ha(3)=subplot(3,1,3);
            plot((test_data.n580.raw(:,8)*180/pi /dt),'.'); hold on;
            plot(LP_15smooth(test_data.n580.raw(:,8)*180/pi /dt),'*')
            grid on
            xlabel('timestep, $k$','Interpreter','latex','FontSize',15)
            ylabel('$r$ $(deg/s)$','Interpreter','latex','FontSize',15)
            
            linkaxes(ha,'x')
            hold off

            disp('zoom plot to desired section of data and hit enter.')
            pause

            disp('Select two sequential data points:')
            [ind,~]=ginput(2);
            id_s_raw = round(ind(1));
            id_e_raw = round(ind(2));
            if id_e_raw < id_s_raw
                error('Datapoints must be sequential')
            end
            close(1)

            t_GNSS_start = test_data.n580.raw(id_s_raw,2);
            t_GNSS_end = test_data.n580.raw(id_e_raw,2);

            [~,id_s_filt] = min(abs(test_data.n580.filt(:,2) - t_GNSS_start));
            [~,id_e_filt] = min(abs(test_data.n580.filt(:,2) - t_GNSS_end));

            [~,i_FMUR_s] = min(abs(test_data.FMUR.sys.time_gnss_s - t_GNSS_start));
            [~,i_FMUR_e] = min(abs(test_data.FMUR.sys.time_gnss_s - t_GNSS_end));
        
        elseif crop_var==5 % RPM
            figure(1),plot(test_data.FMUR.pwr.rpm,'.'); hold on;
            %plot(LP_15smooth(test_data.FMUR.pwr.rpm),'*');
            ylim([0 15000])
            grid on
            xlabel('timestep, $k$','Interpreter','latex','FontSize',15)
            ylabel('$rpm$ $(rev/min)$','Interpreter','latex','FontSize',15)
            title('RPM Time History','FontSize',16,'Interpreter','latex')
            subtitle('select 2 points to crop data','FontSize',15,'Interpreter','latex')
            hold off

            disp('zoom plot to desired section of data and hit enter.')
            pause

            disp('Select two sequential data points:')
            [ind,~]=ginput(2);
            i_FMUR_s = round(ind(1));
            i_FMUR_e = round(ind(2));
            if i_FMUR_e < i_FMUR_s
                error('Datapoints must be sequential')
            end
            close(1)

            t_GNSS_start = test_data.FMUR.sys.time_gnss_s(i_FMUR_s);
            t_GNSS_end = test_data.FMUR.sys.time_gnss_s(i_FMUR_e);

            [~,id_s_raw] = min(abs(test_data.n580.raw(:,2) - t_GNSS_start));
            [~,id_e_raw] = min(abs(test_data.n580.raw(:,2) - t_GNSS_end));

            [~,id_s_filt] = min(abs(test_data.n580.filt(:,2) - t_GNSS_start));
            [~,id_e_filt] = min(abs(test_data.n580.filt(:,2) - t_GNSS_end));

        elseif crop_var==6 % delta_e / delta_a / delta_r
            figure(1)
            ha(1)=subplot(3,1,1);
            plot(test_data.FMUR.adc_volt(:,3),'.'); 
            grid on
            xlabel('timestep, $k$','Interpreter','latex','FontSize',15)
            ylabel('$\delta_e$ $(volts)$','Interpreter','latex','FontSize',15)
            title('Control Surface Time History from pot. ','FontSize',16,'Interpreter','latex')
            subtitle('select 2 points to crop data','FontSize',15,'Interpreter','latex')

            ha(2)=subplot(3,1,2);
            plot((test_data.FMUR.adc_volt(:,5)+test_data.FMUR.adc_volt(:,6))/2,'.'); 
            grid on
            xlabel('timestep, $k$','Interpreter','latex','FontSize',15)
            ylabel('$\delta_a$ $(volts)$','Interpreter','latex','FontSize',15)

            ha(3)=subplot(3,1,3);
            plot(test_data.FMUR.adc_volt(:,4),'.'); hold on;
            grid on
            xlabel('timestep, $k$','Interpreter','latex','FontSize',15)
            ylabel('$\delta_r$ $(volts)$','Interpreter','latex','FontSize',15)
            
            linkaxes(ha,'x')
            hold off

            disp('zoom plot to desired section of data and hit enter.')
            pause

            disp('Select two sequential data points:')
            [ind,~]=ginput(2);
            i_FMUR_s = round(ind(1));
            i_FMUR_e = round(ind(2));
            if i_FMUR_e < i_FMUR_s
                error('Datapoints must be sequential')
            end
            close(1)

            t_GNSS_start = test_data.FMUR.sys.time_gnss_s(i_FMUR_s);
            t_GNSS_end = test_data.FMUR.sys.time_gnss_s(i_FMUR_e);

            [~,id_s_raw] = min(abs(test_data.n580.raw(:,2) - t_GNSS_start));
            [~,id_e_raw] = min(abs(test_data.n580.raw(:,2) - t_GNSS_end));

            [~,id_s_filt] = min(abs(test_data.n580.filt(:,2) - t_GNSS_start));
            [~,id_e_filt] = min(abs(test_data.n580.filt(:,2) - t_GNSS_end));

        elseif crop_var==7 % delta_f
            figure(1),plot((test_data.FMUR.adc_volt(:,8)-test_data.FMUR.adc_volt(:,7))/2,'.'); 
            grid on
            xlabel('timestep, $k$','Interpreter','latex','FontSize',15)
            ylabel('$\delta_f$ $(volts)$','Interpreter','latex','FontSize',15)
            title('Flap Time History from pot. ','FontSize',16,'Interpreter','latex')
            subtitle('select 2 points to crop data','FontSize',15,'Interpreter','latex')

            disp('zoom plot to desired section of data and hit enter.')
            pause

            disp('Select two sequential data points:')
            [ind,~]=ginput(2);
            i_FMUR_s = round(ind(1));
            i_FMUR_e = round(ind(2));
            if i_FMUR_e < i_FMUR_s
                error('Datapoints must be sequential')
            end
            close(1)

            t_GNSS_start = test_data.FMUR.sys.time_gnss_s(i_FMUR_s);
            t_GNSS_end = test_data.FMUR.sys.time_gnss_s(i_FMUR_e);

            [~,id_s_raw] = min(abs(test_data.n580.raw(:,2) - t_GNSS_start));
            [~,id_e_raw] = min(abs(test_data.n580.raw(:,2) - t_GNSS_end));

            [~,id_s_filt] = min(abs(test_data.n580.filt(:,2) - t_GNSS_start));
            [~,id_e_filt] = min(abs(test_data.n580.filt(:,2) - t_GNSS_end));

        elseif crop_var==8 % DELTA_p / p_static
            figure(1)
            ha(1)=subplot(2,1,1);
            plot(test_data.FMUR.pres.diff.pres_pa*0.020885434273039,'.'); 
            grid on
            xlabel('timestep, $k$','Interpreter','latex','FontSize',15)
            ylabel('$\Delta p$ $(lb/ft^2)$','Interpreter','latex','FontSize',15)
            title('Pitot-Static Time History ','FontSize',16,'Interpreter','latex')
            subtitle('select 2 points to crop data','FontSize',15,'Interpreter','latex')

            ha(2)=subplot(2,1,2);
            plot(test_data.FMUR.pres.static.pres_pa*0.020885434273039,'.'); 
            grid on
            xlabel('timestep, $k$','Interpreter','latex','FontSize',15)
            ylabel('$p_{static}$ $(lb/ft^2)$','Interpreter','latex','FontSize',15)

            linkaxes(ha,'x')

            disp('zoom plot to desired section of data and hit enter.')
            pause

            disp('Select two sequential data points:')
            [ind,~]=ginput(2);
            i_FMUR_s = round(ind(1));
            i_FMUR_e = round(ind(2));
            if i_FMUR_e < i_FMUR_s
                error('Datapoints must be sequential')
            end
            close(1)

            t_GNSS_start = test_data.FMUR.sys.time_gnss_s(i_FMUR_s);
            t_GNSS_end = test_data.FMUR.sys.time_gnss_s(i_FMUR_e);

            [~,id_s_raw] = min(abs(test_data.n580.raw(:,2) - t_GNSS_start));
            [~,id_e_raw] = min(abs(test_data.n580.raw(:,2) - t_GNSS_end));

            [~,id_s_filt] = min(abs(test_data.n580.filt(:,2) - t_GNSS_start));
            [~,id_e_filt] = min(abs(test_data.n580.filt(:,2) - t_GNSS_end));

        elseif crop_var==9 % north / east position (n580)
            figure(1)
            ha(1)=subplot(2,1,1);
            plot(test_data.n580.filt(:,5)*3.28084,'.')
            grid on
            xlabel('timestep, $k$','Interpreter','latex','FontSize',15)
            ylabel('East $(ft)$','Interpreter','latex','FontSize',15)
            title('Local Position Time History from n580 ','FontSize',16,'Interpreter','latex')
            subtitle('select 2 points to crop data','FontSize',15,'Interpreter','latex')

            ha(2)=subplot(2,1,2);
            plot(test_data.n580.filt(:,6)*3.28084,'.')
            grid on
            xlabel('timestep, $k$','Interpreter','latex','FontSize',15)
            ylabel('North $(ft)$','Interpreter','latex','FontSize',15)

            linkaxes(ha,'x')

            disp('zoom plot to desired section of data and hit enter.')
            pause

            disp('Select two sequential data points:')
            [ind,~]=ginput(2);
            id_s_filt = round(ind(1));
            id_e_filt = round(ind(2));
            if id_e_filt < id_s_filt
                error('Datapoints must be sequential')
            end
            close(1)

            t_GNSS_start = test_data.n580.filt(id_s_filt,2);
            t_GNSS_end = test_data.n580.filt(id_e_filt,2);

            [~,id_s_raw] = min(abs(test_data.n580.raw(:,2) - t_GNSS_start));
            [~,id_e_raw] = min(abs(test_data.n580.raw(:,2) - t_GNSS_end));

            [~,i_FMUR_s] = min(abs(test_data.FMUR.sys.time_gnss_s - t_GNSS_start));
            [~,i_FMUR_e] = min(abs(test_data.FMUR.sys.time_gnss_s - t_GNSS_end));

        elseif crop_var==10 % alpha / q / delta_e
            figure(1)
            ha(1)=subplot(3,1,1);
            plot(test_data.FMUR.adc_volt(:,2),'.'); 
            grid on
            xlabel('timestep, $k$','Interpreter','latex','FontSize',15)
            ylabel('$\alpha$ $(volts)$','Interpreter','latex','FontSize',15)
            title('alpha / q / elevator Time History from pot. ','FontSize',16,'Interpreter','latex')
            subtitle('select 2 points to crop data','FontSize',15,'Interpreter','latex')

            ha(2)=subplot(3,1,2);
            plot(-test_data.FMUR.imu.gyro_radps(:,2)*180/pi,'.'); 
            grid on
            xlabel('timestep, $k$','Interpreter','latex','FontSize',15)
            ylabel('$q_{FMUR}$ $(deg/s)$','Interpreter','latex','FontSize',15)

            ha(3)=subplot(3,1,3);
            plot(test_data.FMUR.adc_volt(:,3),'.'); hold on;
            grid on
            xlabel('timestep, $k$','Interpreter','latex','FontSize',15)
            ylabel('$\delta_e$ $(volts)$','Interpreter','latex','FontSize',15)
            
            linkaxes(ha,'x')
            hold off

            disp('zoom plot to desired section of data and hit enter.')
            pause

            disp('Select two sequential data points:')
            [ind,~]=ginput(2);
            i_FMUR_s = round(ind(1));
            i_FMUR_e = round(ind(2));
            if i_FMUR_e < i_FMUR_s
                error('Datapoints must be sequential')
            end
            close(1)

            t_GNSS_start = test_data.FMUR.sys.time_gnss_s(i_FMUR_s);
            t_GNSS_end = test_data.FMUR.sys.time_gnss_s(i_FMUR_e);

            [~,id_s_raw] = min(abs(test_data.n580.raw(:,2) - t_GNSS_start));
            [~,id_e_raw] = min(abs(test_data.n580.raw(:,2) - t_GNSS_end));

            [~,id_s_filt] = min(abs(test_data.n580.filt(:,2) - t_GNSS_start));
            [~,id_e_filt] = min(abs(test_data.n580.filt(:,2) - t_GNSS_end));

        elseif crop_var==11 % beta / p / delta_a
            figure(1)
            ha(1)=subplot(3,1,1);
            plot(test_data.FMUR.adc_volt(:,1)*180/pi,'.'); 
            grid on
            xlabel('timestep, $k$','Interpreter','latex','FontSize',15)
            ylabel('$\beta$ $(volts)$','Interpreter','latex','FontSize',15)
            title('beta / p / aileron Time History from pot. ','FontSize',16,'Interpreter','latex')
            subtitle('select 2 points to crop data','FontSize',15,'Interpreter','latex')

            ha(2)=subplot(3,1,2);
            plot(-test_data.FMUR.imu.gyro_radps(:,1)*180/pi,'.'); 
            grid on
            xlabel('timestep, $k$','Interpreter','latex','FontSize',15)
            ylabel('$p_{FMUR}$ $(deg/s)$','Interpreter','latex','FontSize',15)

            ha(3)=subplot(3,1,3);
            plot((test_data.FMUR.adc_volt(:,5)+test_data.FMUR.adc_volt(:,6))/2,'.'); hold on;
            grid on
            xlabel('timestep, $k$','Interpreter','latex','FontSize',15)
            ylabel('$\delta_a$ $(volts)$','Interpreter','latex','FontSize',15)
            
            linkaxes(ha,'x')
            hold off

            disp('zoom plot to desired section of data and hit enter.')
            pause

            disp('Select two sequential data points:')
            [ind,~]=ginput(2);
            i_FMUR_s = round(ind(1));
            i_FMUR_e = round(ind(2));
            if i_FMUR_e < i_FMUR_s
                error('Datapoints must be sequential')
            end
            close(1)

            t_GNSS_start = test_data.FMUR.sys.time_gnss_s(i_FMUR_s);
            t_GNSS_end = test_data.FMUR.sys.time_gnss_s(i_FMUR_e);

            [~,id_s_raw] = min(abs(test_data.n580.raw(:,2) - t_GNSS_start));
            [~,id_e_raw] = min(abs(test_data.n580.raw(:,2) - t_GNSS_end));

            [~,id_s_filt] = min(abs(test_data.n580.filt(:,2) - t_GNSS_start));
            [~,id_e_filt] = min(abs(test_data.n580.filt(:,2) - t_GNSS_end));

        elseif crop_var==12 % beta / r / delta_r
            figure(1)
            ha(1)=subplot(3,1,1);
            plot(test_data.FMUR.adc_volt(:,1),'.'); 
            grid on
            xlabel('timestep, $k$','Interpreter','latex','FontSize',15)
            ylabel('$\beta$ $(volts)$','Interpreter','latex','FontSize',15)
            title('beta / r / rudder Time History ','FontSize',16,'Interpreter','latex')
            subtitle('select 2 points to crop data','FontSize',15,'Interpreter','latex')

            ha(2)=subplot(3,1,2);
            plot(-test_data.FMUR.imu.gyro_radps(:,3)*180/pi,'.'); 
            grid on
            xlabel('timestep, $k$','Interpreter','latex','FontSize',15)
            ylabel('$r_{FMUR}$ $(deg/s)$','Interpreter','latex','FontSize',15)

            ha(3)=subplot(3,1,3);
            plot(test_data.FMUR.adc_volt(:,4),'.'); hold on;
            grid on
            xlabel('timestep, $k$','Interpreter','latex','FontSize',15)
            ylabel('$\delta_r$ $(volts)$','Interpreter','latex','FontSize',15)
            
            linkaxes(ha,'x')
            hold off

            disp('zoom plot to desired section of data and hit enter.')
            pause

            disp('Select two sequential data points:')
            [ind,~]=ginput(2);
            i_FMUR_s = round(ind(1));
            i_FMUR_e = round(ind(2));
            if i_FMUR_e < i_FMUR_s
                error('Datapoints must be sequential')
            end
            close(1)

            t_GNSS_start = test_data.FMUR.sys.time_gnss_s(i_FMUR_s);
            t_GNSS_end = test_data.FMUR.sys.time_gnss_s(i_FMUR_e);

            [~,id_s_raw] = min(abs(test_data.n580.raw(:,2) - t_GNSS_start));
            [~,id_e_raw] = min(abs(test_data.n580.raw(:,2) - t_GNSS_end));

            [~,id_s_filt] = min(abs(test_data.n580.filt(:,2) - t_GNSS_start));
            [~,id_e_filt] = min(abs(test_data.n580.filt(:,2) - t_GNSS_end));

        elseif crop_var==13 % rpm / ax / Delta_p
            figure(1)
            ha(1)=subplot(3,1,1);
            plot(test_data.FMUR.pwr.rpm,'.'); 
            ylim([0 15000])
            grid on
            xlabel('timestep, $k$','Interpreter','latex','FontSize',15)
            ylabel('$rpm$','Interpreter','latex','FontSize',15)
            title('rpm / ax / Delta p Time History ','FontSize',16,'Interpreter','latex')
            subtitle('select 2 points to crop data','FontSize',15,'Interpreter','latex')

            ha(2)=subplot(3,1,2);
            plot(-test_data.FMUR.imu.accel_mps2(:,1)*3.28084,'.'); hold on
            plot(LP_15smooth(-test_data.FMUR.imu.accel_mps2(:,1)*3.28084),'*');
            grid on
            xlabel('timestep, $k$','Interpreter','latex','FontSize',15)
            ylabel('$a_{x_{FMUR}}$ $(ft/s^2)$','Interpreter','latex','FontSize',15)

            ha(3)=subplot(3,1,3);
            plot(test_data.FMUR.pres.diff.pres_pa*0.020885434273039,'.'); hold on;
            grid on
            xlabel('timestep, $k$','Interpreter','latex','FontSize',15)
            ylabel('$\Delta p$ $(lb/ft^2)$','Interpreter','latex','FontSize',15)
            
            linkaxes(ha,'x')
            hold off

            disp('zoom plot to desired section of data and hit enter.')
            pause

            disp('Select two sequential data points:')
            [ind,~]=ginput(2);
            i_FMUR_s = round(ind(1));
            i_FMUR_e = round(ind(2));
            if i_FMUR_e < i_FMUR_s
                error('Datapoints must be sequential')
            end
            close(1)

            t_GNSS_start = test_data.FMUR.sys.time_gnss_s(i_FMUR_s);
            t_GNSS_end = test_data.FMUR.sys.time_gnss_s(i_FMUR_e);

            [~,id_s_raw] = min(abs(test_data.n580.raw(:,2) - t_GNSS_start));
            [~,id_e_raw] = min(abs(test_data.n580.raw(:,2) - t_GNSS_end));

            [~,id_s_filt] = min(abs(test_data.n580.filt(:,2) - t_GNSS_start));
            [~,id_e_filt] = min(abs(test_data.n580.filt(:,2) - t_GNSS_end));

        else
            error('Must select one of the variables')
        end
            if t_GNSS_start < 1e3
                warning('No GPS lock at starting crop time.')
            end
            if t_GNSS_end < 1e3
                warning('No GPS lock at ending crop time.')
            end

            % logic for n580 indexing (if number of timesteps not the same)
            N_filt = id_e_filt - id_s_filt + 1;
            N_raw = id_e_raw - id_s_raw + 1;
            if N_filt > N_raw
                id_e_filt = id_s_filt + N_raw - 1;
                warning('n580 N_raw < N_filt')
            elseif N_raw > N_filt
                id_e_raw = id_s_raw + N_filt - 1;
                warning('n580 N_raw > N_filt')
            end

            % logic for FMUR indexing
            N_FMUR = i_FMUR_e - i_FMUR_s + 1;
            if N_FMUR < N_raw
                warning('N_FMUR < N_n580. FMUR and n580 data do not align.')
                id_e_filt = id_s_filt + N_FMUR - 1;
                id_e_raw = id_s_raw + N_FMUR -1;
            elseif N_FMUR > N_raw
                warning('N_FMUR > N_n580. FMUR and n580 data do not align.')
                i_FMUR_e = i_FMUR_s + N_raw -1;
            end


            disp(append('Cropped maneuver jj = ',num2str(jj)))
            t_GNSS_se(jj,:) = [t_GNSS_start, t_GNSS_end];
            i_FMUR_se(jj,:) = [i_FMUR_s, i_FMUR_e];
            i_n580_raw_se(jj,:) = [id_s_raw, id_e_raw];
            i_n580_filt_se(jj,:) = [id_s_filt, id_e_filt];
            Man_Lab = input('Label maneuver: ','s');
            maneuver_label(jj,1) = string(Man_Lab);
        end
        
    else
        disp('Automated cropping starting.')
        
        % Find FMU-R time index when GNSS time starts
        % assuming we have a GPS lock at the final timestep
        i_FMUR_e = length(test_data.FMUR.sys.time_gnss_s); 
        for ii=2:i_FMUR_e
            if abs(test_data.FMUR.sys.time_gnss_s(ii-1)-test_data.FMUR.sys.time_gnss_s(ii)) > 1e3
                i_FMUR_s = ii;
                break
            end
        end

        t_GNSS_start = test_data.FMUR.sys.time_gnss_s(i_FMUR_s);
        t_GNSS_end = test_data.FMUR.sys.time_gnss_s(i_FMUR_e);

        [~,id_s_raw] = min(abs(test_data.n580.raw(:,2) - t_GNSS_start));
        [~,id_e_raw] = min(abs(test_data.n580.raw(:,2) - t_GNSS_end));

        [~,id_s_filt] = min(abs(test_data.n580.filt(:,2) - t_GNSS_start));
        [~,id_e_filt] = min(abs(test_data.n580.filt(:,2) - t_GNSS_end));

        % logic for n580 indexing (if number of timesteps not the same)
        N_filt = id_e_filt - id_s_filt + 1;
        N_raw = id_e_raw - id_s_raw + 1;
        if N_filt > N_raw
            id_e_filt = id_s_filt + N_raw - 1;
            warning('n580 N_raw < N_filt')
        elseif N_raw > N_filt
            id_e_raw = id_s_raw + N_filt - 1;
            warning('n580 N_raw > N_filt')
        end

        % logic for FMUR indexing
        N_FMUR = i_FMUR_e - i_FMUR_s + 1;
        if N_FMUR < N_raw
            warning('N_FMUR < N_n580. FMUR and n580 data do not align.')
            id_e_filt = id_s_filt + N_FMUR - 1;
            id_e_raw = id_s_raw + N_FMUR -1;
        elseif N_FMUR > N_raw
            warning('N_FMUR > N_n580. FMUR and n580 data do not align.')
            i_FMUR_e = i_FMUR_s + N_raw -1;
        end

        t_GNSS_se = [t_GNSS_start, t_GNSS_end];
        i_FMUR_se = [i_FMUR_s, i_FMUR_e];
        i_n580_raw_se = [id_s_raw, id_e_raw];
        i_n580_filt_se = [id_s_filt, id_e_filt];
        maneuver_label = [];
    end
end