% crop_IMU_data.fcn crops and computes accel and gyro biases and estimated
% noise statistics. The function will ask the user crop appropriate
% time histories of the n580 and FMUR when the aircraft is motionless.
%
% [b_a,b_g,sig_a,sig_g,i_FMUR_se,i_n580_raw_se,i_n580_filt_se,t_GNSS_se]=crop_IMU_bias(test_data,theta_0)
%
% INPUTS: 
%   test_data: data structure from read_flight_test_data.fcn
%
% OPTIONAL INPUTS:
%   theta_0: pitch attitude during data crop (default will be 7deg nose
%       high for IBIS). This will be used to estimate the gravity vector
%
% OUTPUTS:
%   b_a: data structure of accel biases in ft/s^2
%       .FMUR
%       .n580
%   b_g: data structure of gyro biases in deg/s
%       .FMUR
%       .n580
%   sig_a: data structure of accel noise standard deviation (ft/s^2)
%       .FMUR
%       .n580
%   sig_g: data structure of gyro noise standard deviation (deg/s)
%       .FMUR
%       .n580
%
% OPTIONAL OUTPUTS
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
%   t_GNSS_se: matrix of GNSS times for the cropping data. 
%       The first column corresponds to the starting time, the second 
%       column corresponds to the ending time. The rows correspond to a 
%       particular maneuver.
%
% Sam Jaeger
% jaege246@umn.edu
% 7/1/2026

function [b_a,b_g,sig_a,sig_g,varargout]=crop_IMU_bias(test_data,varargin)
    % acceleration due to gravity
    g = 32.174;
    % IBIS normally sits at 7 degrees nose high
    if nargin == 2
        theta_0 = varargin{1};
    else
        theta_0 = 7; % deg
    end

    % initalize data structures
    t_GNSS_se = [];
    i_FMUR_se = [];
    i_n580_raw_se = [];
    i_n580_filt_se = [];

    add_maneuver = true;
    for jj=1:10 % loop over data cropping 
        
        if jj>1
            disp('Add motionless data crop?')
            add_maneuver = input('true / false:   ');
        end
    
        if add_maneuver == false
            break
        end

        % n580 IMU---------------------------------------------------------
        dt= 0.01; % for n580 that UMN has
        figure(1); hold on
        ha(1)=subplot(2,1,1);
        plot((-test_data.n580.raw(:,13)*3.28084 /dt),'.');  hold on
        plot((test_data.n580.raw(:,12)*3.28084 /dt),'.');
        plot((test_data.n580.raw(:,11)*3.28084 /dt),'.');
        grid on
        xlabel('timestep, $k$','Interpreter','latex','FontSize',15)
        ylabel('$a_{xyz}$ $(ft/s^2)$','Interpreter','latex','FontSize',15)
        title('IMU Time History from n580 ','FontSize',16,'Interpreter','latex')
        subtitle('select 2 points to crop data','FontSize',15,'Interpreter','latex')

        ha(2)=subplot(2,1,2);
        plot((-test_data.n580.raw(:,10)*180/pi /dt),'.'); hold on
        plot((test_data.n580.raw(:,9)*180/pi /dt),'.');
        plot((test_data.n580.raw(:,8)*180/pi /dt),'.');
        grid on
        xlabel('timestep, $k$','Interpreter','latex','FontSize',15)
        ylabel('$\omega$ $(deg/s)$','Interpreter','latex','FontSize',15)

        linkaxes(ha,'x')
        hold off

        disp('zoom plot to desired section of data and hit enter.')
        pause

        disp('Select two sequential motionless data points:')
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

        % accel bias, subtract off gravity
        b_a.n580(1,jj) = mean((-test_data.n580.raw(id_s_raw:id_e_raw,13)*3.28084 /dt)-g*sind(theta_0));
        b_a.n580(2,jj) = mean((test_data.n580.raw(id_s_raw:id_e_raw,12)*3.28084 /dt));
        b_a.n580(3,jj) = mean((test_data.n580.raw(id_s_raw:id_e_raw,11)*3.28084 /dt)+g*cosd(theta_0));

        % gyro bias
        b_g.n580(1,jj) = mean((-test_data.n580.raw(id_s_raw:id_e_raw,10)*180/pi /dt));
        b_g.n580(2,jj) = mean((test_data.n580.raw(id_s_raw:id_e_raw,9)*180/pi /dt));
        b_g.n580(3,jj) = mean((test_data.n580.raw(id_s_raw:id_e_raw,8)*180/pi /dt)); 

        % accel noise
        sig_a.n580(1,jj) = std((-test_data.n580.raw(id_s_raw:id_e_raw,13)*3.28084 /dt));
        sig_a.n580(2,jj) = std((test_data.n580.raw(id_s_raw:id_e_raw,12)*3.28084 /dt));
        sig_a.n580(3,jj) = std((test_data.n580.raw(id_s_raw:id_e_raw,11)*3.28084 /dt));

        % gyro noise
        sig_g.n580(1,jj) =  std((-test_data.n580.raw(id_s_raw:id_e_raw,10)*180/pi /dt));
        sig_g.n580(2,jj) =  std((test_data.n580.raw(id_s_raw:id_e_raw,9)*180/pi /dt));
        sig_g.n580(3,jj) =  std((test_data.n580.raw(id_s_raw:id_e_raw,8)*180/pi /dt)); 

        % FMUR IMU---------------------------------------------------------
        figure(1); hold on
        ha(1)=subplot(2,1,1);
        plot(-test_data.FMUR.imu.accel_mps2(:,1)*3.28084,'.'); hold on
        plot(-test_data.FMUR.imu.accel_mps2(:,2)*3.28084,'.');
        plot(-test_data.FMUR.imu.accel_mps2(:,3)*3.28084,'.');
        grid on
        xlabel('timestep, $k$','Interpreter','latex','FontSize',15)
        ylabel('$a_{xyz}$ $(ft/s^2)$','Interpreter','latex','FontSize',15)
        title('IMU Time History from FMUR ','FontSize',16,'Interpreter','latex')
        subtitle('select 2 points to crop data','FontSize',15,'Interpreter','latex')

        ha(2)=subplot(2,1,2);
        plot(-test_data.FMUR.imu.gyro_radps(:,1)*180/pi,'.');  hold on 
        plot(-test_data.FMUR.imu.gyro_radps(:,2)*180/pi,'.'); 
        plot(-test_data.FMUR.imu.gyro_radps(:,3)*180/pi,'.'); 
        grid on
        xlabel('timestep, $k$','Interpreter','latex','FontSize',15)
        ylabel('$\omega$ $(deg/s)$','Interpreter','latex','FontSize',15)

        linkaxes(ha,'x')
        hold off

        disp('zoom plot to desired section of data and hit enter.')
        pause

        disp('Select two sequential motionless data points:')
        [ind,~]=ginput(2);
        i_FMUR_s = round(ind(1));
        i_FMUR_e = round(ind(2));
        if i_FMUR_e < i_FMUR_s
            error('Datapoints must be sequential')
        end
        close(1)

        % accel bias, subtract off gravity
        b_a.FMUR(1,jj) = mean((-test_data.FMUR.imu.accel_mps2(i_FMUR_s:i_FMUR_e,1)*3.28084)-g*sind(theta_0));
        b_a.FMUR(2,jj) = mean((-test_data.FMUR.imu.accel_mps2(i_FMUR_s:i_FMUR_e,2)*3.28084));
        b_a.FMUR(3,jj) = mean((-test_data.FMUR.imu.accel_mps2(i_FMUR_s:i_FMUR_e,3)*3.28084)+g*cosd(theta_0));

        % gyro bias
        b_g.FMUR(1,jj) = mean(-test_data.FMUR.imu.gyro_radps(i_FMUR_s:i_FMUR_e,1)*180/pi);
        b_g.FMUR(2,jj) = mean(-test_data.FMUR.imu.gyro_radps(i_FMUR_s:i_FMUR_e,2)*180/pi);
        b_g.FMUR(3,jj) = mean(-test_data.FMUR.imu.gyro_radps(i_FMUR_s:i_FMUR_e,3)*180/pi); 

        % accel noise
        sig_a.FMUR(1,jj) = std(-test_data.FMUR.imu.accel_mps2(i_FMUR_s:i_FMUR_e,1)*3.28084);
        sig_a.FMUR(2,jj) = std(-test_data.FMUR.imu.accel_mps2(i_FMUR_s:i_FMUR_e,2)*3.28084);
        sig_a.FMUR(3,jj) = std(-test_data.FMUR.imu.accel_mps2(i_FMUR_s:i_FMUR_e,3)*3.28084);

        % gyro noise
        sig_g.FMUR(1,jj) =  std(-test_data.FMUR.imu.gyro_radps(i_FMUR_s:i_FMUR_e,1)*180/pi);
        sig_g.FMUR(2,jj) =  std(-test_data.FMUR.imu.gyro_radps(i_FMUR_s:i_FMUR_e,2)*180/pi);
        sig_g.FMUR(3,jj) =  std(-test_data.FMUR.imu.gyro_radps(i_FMUR_s:i_FMUR_e,3)*180/pi); 


        disp(append('Cropped maneuver jj = ',num2str(jj)))
        t_GNSS_se(jj,:) = [t_GNSS_start, t_GNSS_end];
        i_FMUR_se(jj,:) = [i_FMUR_s, i_FMUR_e];
        i_n580_raw_se(jj,:) = [id_s_raw, id_e_raw];
        i_n580_filt_se(jj,:) = [id_s_filt, id_e_filt];
    end
    varargout{1} = i_FMUR_se;
    varargout{2} = i_n580_raw_se;
    varargout{3} = i_n580_filt_se;
    varargout{4} = t_GNSS_se;
end