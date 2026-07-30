% align_gyro_thist.fcn attempts to time sync the n580 to the FMU-R data 
% based on gyro responses. This is done to avoid using GPS time tagging. 
% This function relies on the built in "alignsignals.fcn" in MATLAB.
%
% [D, i_FMUR_se, i_n580_raw_se,i_n580_filt_se] = align_gyro_thist(test_data, i_FMUR_se, plt_synced_response, t_buffer)
%
% INPUTS:
%   test_data: data structure from flight test
%
% OPTIONAL INPUTS:
%   i_FMUR_se: starting and ending index of FMUR data
%   plt_synced_response: true / false to plot synced time response.
%   t_buffer: 2 element vector of buffer times in seconds
%
% OUTPUTS:
%   D: integer index of the time delay of the n580 from the FMU-R.
%   i_FMUR_se: 2 element vector of synced cropping indicies for the FMUR 
%       data. The first entry corresponds to the starting index, the second 
%       entry corresponds to the ending index.
%   i_n580_raw_se: 2 element vector of synced cropping indicies for the raw
%       n580 data. The first column corresponds to the starting index, the
%       second column corresponds to the ending index. 
%   i_n580_filt_se: 2 element vector of synced cropping indicies for the 
%       filtered n580 data. The first column corresponds to the starting 
%       index, the second column corresponds to the ending index.
%
% Sam Jaeger
% jaege246@umn.edu
% 7/30/2026

function [D, i_FMUR_se, i_n580_raw_se,i_n580_filt_se] = align_gyro_thist(test_data,varargin)
    
    narginchk(1,4)
    if nargin == 1 % no starting and ending index
        i_FMUR_s = 1;
        i_FMUR_e = length(test_data.FMUR.adc_volt(:,1));
        
        id_s_raw = 1;
        id_e_raw = length(test_data.n580.raw(:,1));

        plt_synced_response = true;
    elseif nargin == 2 % FMUR starting index is provided
        i_FMUR_se = varargin{1};
        i_FMUR_s = i_FMUR_se(1);
        i_FMUR_e = i_FMUR_se(2);

        % add 5 sec buffer on either side of the FMU-R data crop
        id_s_raw = i_FMUR_s - 8*100;
        id_e_raw = i_FMUR_e + 5*100;
        
        plt_synced_response = true;
    elseif nargin == 3 % FMUR starting index is provided and plot sync
        i_FMUR_se = varargin{1};
        i_FMUR_s = i_FMUR_se(1);
        i_FMUR_e = i_FMUR_se(2);

        % add buffer on either side of the FMU-R data crop
        id_s_raw = i_FMUR_s - 8*100; 
        id_e_raw = i_FMUR_e + 5*100;

        plt_synced_response = varargin{2};
    elseif nargin == 4 % FMUR starting index is provided, plot sync, and buffer time
        i_FMUR_se = varargin{1};
        i_FMUR_s = i_FMUR_se(1);
        i_FMUR_e = i_FMUR_se(2);

        % add buffer on either side of the FMU-R data crop
        t_buffer = varargin{3};
        id_s_raw = i_FMUR_s - round(t_buffer(1))*100; 
        id_e_raw = i_FMUR_e + round(t_buffer(2))*100;

        plt_synced_response = varargin{2};

    end


    dt = 0.01;
    % n580 gyro readings
    %   x and z flipped, x negative
    p_n = -test_data.n580.raw(id_s_raw:id_e_raw,10)*180/pi /dt; % deg/s
    q_n = test_data.n580.raw(id_s_raw:id_e_raw,9)*180/pi /dt; % deg/s
    r_n = test_data.n580.raw(id_s_raw:id_e_raw,8)*180/pi /dt; % deg/s

    % FMU-R gyro readings
    p_F = -test_data.FMUR.imu.gyro_radps(i_FMUR_s:i_FMUR_e,1)*180/pi;
    q_F = -test_data.FMUR.imu.gyro_radps(i_FMUR_s:i_FMUR_e,2)*180/pi;
    r_F = -test_data.FMUR.imu.gyro_radps(i_FMUR_s:i_FMUR_e,3)*180/pi;

    figure,plot(q_F,'*'); hold on; plot(q_n,'.')

    % align gyro data
    [p_Fa,p_na,D_p] = alignsignals(p_F,p_n, Method='xcorr');
    [q_Fa,q_na,D_q] = alignsignals(q_F,q_n, Method='xcorr');
    [r_Fa,r_na,D_r] = alignsignals(r_F,r_n, Method='xcorr');
    disp(append('N_p delay = ',num2str(D_p)));
    disp(append('N_q delay = ',num2str(D_q)));
    disp(append('N_r delay = ',num2str(D_r)));
    

    if D_p == D_q && D_p == D_r
        disp('all delays match.')
        D = D_p; 
    elseif D_p == D_q
        disp('N_p & N_q match.')
        D = D_p;
    elseif D_p == D_r
        disp('N_p & N_r match.')
        D = D_p;
    elseif D_q == D_r
        disp('N_q & N_r match.')
        D = D_q;
    else % none match
        disp('no delays match, taking rounded average.')
        D  = round(mean([D_p, D_q, D_r]));
    end

    % set indices
    if D > 0 % n580 ahead of FMU-R
        i_FMUR_s = i_FMUR_s - D;
        i_FMUR_e = i_FMUR_e - D;
        
        N_FMUR = i_FMUR_e - i_FMUR_s + 1;
        i_n580_raw_se = [id_s_raw (id_s_raw + (N_FMUR-1))];

        t_GNSS_start = test_data.n580.raw(i_n580_raw_se(1),2);
        t_GNSS_end = test_data.n580.raw(i_n580_raw_se(2),2);

        if t_GNSS_start < 1e3
            warning('No GPS lock at starting crop time.')
        end
        if t_GNSS_end < 1e3
            warning('No GPS lock at ending crop time.')
        end

        [~,id_s_filt] = min(abs(test_data.n580.filt(:,2) - t_GNSS_start));
        [~,id_e_filt] = min(abs(test_data.n580.filt(:,2) - t_GNSS_end));

        % logic for n580 indexing (if number of timesteps not the same)
        N_filt = id_e_filt - id_s_filt + 1;
        N_raw = i_n580_raw_se(2) - i_n580_raw_se(1) + 1;
        if N_filt > N_raw
            %id_e_filt = id_s_filt + N_raw - 1;
            id_s_filt = id_e_filt - N_raw + 1;
            disp(append('N_filt = ',num2str(N_filt)))
            disp(append('N_raw = ',num2str(N_raw)))
            warning('n580 N_raw < N_filt')
        end

        i_n580_filt_se = [id_s_filt, id_e_filt];

    elseif D < 0 % FMU-R data ahead of n580
        N_FMUR = i_FMUR_e - i_FMUR_s + 1;
        i_n580_raw_se = [id_s_raw+D (id_s_raw + D + (N_FMUR-1))];
        %N_raw = i_n580_raw_se(2) - i_n580_raw_se(1) + 1;

        t_GNSS_start = test_data.n580.raw(i_n580_raw_se(1),2);
        t_GNSS_end = test_data.n580.raw(i_n580_raw_se(2),2);

        if t_GNSS_start < 1e3
            warning('No GPS lock at starting crop time.')
        end
        if t_GNSS_end < 1e3
            warning('No GPS lock at ending crop time.')
        end

        [~,id_s_filt] = min(abs(test_data.n580.filt(:,2) - t_GNSS_start));
        [~,id_e_filt] = min(abs(test_data.n580.filt(:,2) - t_GNSS_end));

        % logic for n580 indexing (if number of timesteps not the same)
        N_filt = id_e_filt - id_s_filt + 1;
        N_raw = i_n580_raw_se(2) - i_n580_raw_se(1) + 1;
        if N_filt > N_raw
            %id_e_filt = id_s_filt + N_raw - 1;
            id_s_filt = id_e_filt - N_raw + 1;
            disp(append('N_filt = ',num2str(N_filt)))
            disp(append('N_raw = ',num2str(N_raw)))
            warning('n580 N_raw < N_filt')
        end

        i_n580_filt_se = [id_s_filt, id_e_filt];
    else
        disp('data aligned')
        i_n580_raw_se = [];
        i_n580_filt_se = [];
    end

   
    % plot synced gyro data
    if plt_synced_response == true
        if ishandle(31)
            close(31)
        end

        figure(31)
        hi(1)=subplot(3,1,1);
        % plot(p_Fa,'*'); hold on
        % plot(p_na,'.')
        plot(-test_data.FMUR.imu.gyro_radps(i_FMUR_s:i_FMUR_e,1)*180/pi,'*'); hold on
        plot(-test_data.n580.raw(i_n580_raw_se(1):i_n580_raw_se(2),10)*180/pi /dt,'.')
        grid on
        xlabel('$k$ ','Interpreter','latex','FontSize',15)
        ylabel('$p$ $(deg/s)$','Interpreter','latex','FontSize',15)
        title('Aligned gyro data ','FontSize',16,'Interpreter','latex')
        legend('FMU-R','n580')
    
        hi(2)=subplot(3,1,2);
        % plot(q_Fa,'*'); hold on
        % plot(q_na,'.')
        plot(-test_data.FMUR.imu.gyro_radps(i_FMUR_s:i_FMUR_e,2)*180/pi,'*'); hold on
        plot(test_data.n580.raw(i_n580_raw_se(1):i_n580_raw_se(2),9)*180/pi /dt,'.')
        grid on
        xlabel('$k$ ','Interpreter','latex','FontSize',15)
        ylabel('$q$ $(deg/s)$','Interpreter','latex','FontSize',15)
    
        hi(3)=subplot(3,1,3);
        % plot(r_Fa,'*'); hold on;
        % plot(r_na,'.');
        plot(-test_data.FMUR.imu.gyro_radps(i_FMUR_s:i_FMUR_e,3)*180/pi,'*'); hold on
        plot(test_data.n580.raw(i_n580_raw_se(1):i_n580_raw_se(2),8)*180/pi /dt,'.')
        grid on
        xlabel('$k$ ','Interpreter','latex','FontSize',15)
        ylabel('$r$ $(deg/s)$','Interpreter','latex','FontSize',15)
        
        linkaxes(hi,'x')
        hold off
    end
end