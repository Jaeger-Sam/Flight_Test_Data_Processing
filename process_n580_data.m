% process_n580_data formats the n580 data into a data structure with
% consistent timing with respect to data out of the FMU-R. The default is
% to provide starting and ending GNSS times. Alternatively, indices of the
% associated with the raw and filtered time histories can be provided.
%
% data_n580 = process_n580_data(test_data, t_GNSS_start,t_GNSS_end,i_n580_raw_se,i_n580_filt_se)
% 
% INPUTS:
%   test_data: data structure with n580 data from read_flight_test_data
%   t_GNSS_start: starting GNSS time
%   t_GNSS: ending GNSS time
%
% OPTIONAL_INPUTS:
%   i_n580_raw_se: row of starting and ending index of corresponding to the
%                   raw n580 data.
%   i_n580_filt_se: row of starting and ending index corresponding to the
%                   filtered n580 data.
%
% OUTPUTS:
%   data_n580: data structure formatted n580 data
%       .t_GNSS_start_end: starting and ending GNSS times
%       .i_n580_raw_se: starting and ending raw time series index
%       .i_n580_filt_se: starting and ending filtered time series index
%       .t_raw: time vector associated with "raw" n580 data
%       .t_filt: time vector associated with "filtered" n580 data
%       .phi: roll angle (deg)
%       .theta: pitch angle (deg)
%       .psi: heading (deg)
%       .p: body fixed x angular velocity in deg/s
%       .q: body fixed y angular velocity in deg/s
%       .r: body fixed z angular velocity in deg/s
%       .ax: body fixed x acceleration in ft/s^2
%       .ay: body fixed y acceleration in ft/s^2
%       .az: body fixed z acceleration in ft/s^2
%       .easting: position east of inital position in feet
%       .northing: position north of inital position in feet
%       .PHI_PSI_H: latitude-longitude-altitude (MSL) in deg and ft
%       .PHI_PSI_H_std: latitude-longitude-altitude (MSL) standard 
%                       deviation in deg and ft
%       .V: north-east-down inertial velocity in ft/s
%       .Velocity_i: total inertial velocity ft/s
%   
% Sam Jaeger
% jaege246@umn.edu
% 11/4/2025
% Revised: 7/1/2026


function data_n580 = process_n580_data(test_data, t_GNSS_start, t_GNSS_end, varargin)
    dt = 0.01; % fixed at 100 Hz for n580 UMN has

    narginchk(3,5)
    if nargin == 3
        [~,id_s_raw] = min(abs(test_data.n580.raw(:,2) - t_GNSS_start));
        [~,id_e_raw] = min(abs(test_data.n580.raw(:,2) - t_GNSS_end));
    
        [~,id_s_filt] = min(abs(test_data.n580.filt(:,2) - t_GNSS_start));
        [~,id_e_filt] = min(abs(test_data.n580.filt(:,2) - t_GNSS_end));
    else
        i_n580_raw_se = varargin{1};
        i_n580_filt_se = varargin{2};
        id_s_raw = i_n580_raw_se(1);
        id_e_raw = i_n580_raw_se(2);
        id_s_filt = i_n580_filt_se(1);
        id_e_filt = i_n580_filt_se(2);
    end


    t_raw = test_data.n580.raw(id_s_raw:id_e_raw,2) - test_data.n580.raw(id_s_raw,2);
    t_filt = test_data.n580.filt(id_s_filt:id_e_filt,2) - test_data.n580.filt(id_s_filt,2);

    % time
    data_n580.t_GNSS_start_end = [t_GNSS_start, t_GNSS_end];
    data_n580.i_n580_raw_se= [id_s_raw, id_e_raw];
    data_n580.i_n580_filt_se = [id_s_filt, id_e_filt];
    data_n580.t_raw = t_raw;
    data_n580.t_filt = t_filt;
    
    % attitude in deg, in case frame
    phi_c = test_data.n580.raw(id_s_raw:id_e_raw,7)*180/pi;
    theta_c = test_data.n580.raw(id_s_raw:id_e_raw,6)*180/pi;
    psi_c = test_data.n580.raw(id_s_raw:id_e_raw,5)*180/pi;

    % attitude in body frame (rotate in z 180 deg)
    data_n580.phi = -phi_c;
    data_n580.theta = -theta_c;
    data_n580.psi = -psi_c + 180; % defined [0 360]
    for ii=1:length(data_n580.psi) % convert heading between [-180 +180]
        if data_n580.psi(ii) > 180
            data_n580.psi(ii) = data_n580.psi(ii) - 360;
        end
    end

    % gyro readings
    %   x and z flipped, x negative
    data_n580.p = -test_data.n580.raw(id_s_raw:id_e_raw,10)*180/pi /dt; % deg/s
    data_n580.q = test_data.n580.raw(id_s_raw:id_e_raw,9)*180/pi /dt; % deg/s
    data_n580.r = test_data.n580.raw(id_s_raw:id_e_raw,8)*180/pi /dt; % deg/s

    % accel readings
    %   x and z flipped, x negative
    data_n580.ax = -test_data.n580.raw(id_s_raw:id_e_raw,13)*3.28084 /dt; % ft/s^2 
    data_n580.ay = test_data.n580.raw(id_s_raw:id_e_raw,12)*3.28084 /dt; % ft/s^2
    data_n580.az = test_data.n580.raw(id_s_raw:id_e_raw,11)*3.28084 /dt; % ft/s^2

    % easting-northing in ft
    data_n580.easting = test_data.n580.filt(id_s_filt:id_e_filt,5)*3.28084; 
    %   north data is negative for some reason
    data_n580.northing = -test_data.n580.filt(id_s_filt:id_e_filt,6)*3.28084; 

    % latitude,longitude altitude in deg & ft
    data_n580.PHI_PSI_H = [test_data.n580.filt(id_s_filt:id_e_filt,7)*180/pi,...
        test_data.n580.filt(id_s_filt:id_e_filt,8)*180/pi,...
        test_data.n580.filt(id_s_filt:id_e_filt,9)*3.28084]; 

    data_n580.PHI_PSI_H_std= [test_data.n580.filt(id_s_filt:id_e_filt,10)*180/pi,...
        test_data.n580.filt(id_s_filt:id_e_filt,11)*180/pi,...
        test_data.n580.filt(id_s_filt:id_e_filt,12)*3.28084];

    % velocity 
    data_n580.V = test_data.n580.filt(id_s_filt:id_e_filt,13:15)*3.28084; % ft/s
    %data_n580.V = [data_n580.V, test_data.n580.filt(id_s_filt:id_e_filt,15)*3.28084]; % ft/s

    data_n580.Velocity_i = vecnorm(data_n580.V,2,2);
end