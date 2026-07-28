% read_flight_test_data.fcn reads in the .mat data file generated from the
% mat_converter function of the FMU-R. Additionally, the function reads in
% csv data files from the Honeywell n580 if the user request. The script 
% will ask for the file location of all of the needed files. Documentation
% for the FMU-R  can be found here: https://github.com/bolderflight/spaaro
% Will ask the user to also crop, process, and save the data structure.
%
% test_data = read_flight_test_data  
%
% OUTPUTS:
%   test_data
%       .FMUR - data from the FMUR
%           .adc_volt: data from the analog to digital converter which are 
%                potentiometers
%           .gnss - data from GNSS module
%               .alt_msl_m: 
%                   Altitude above Mean Sea Level (MSL), m
%               .alt_wgs84_m: 
%                   Altitude above the WGS84 ellipsoid, m
%               .fix: 
%                   the GNSS fix type - 1 (no fix), 2 (2d fix), 3 (3d
%                   fix), 4 (3d fix with differential GNSS), 5 (3d fix, RTK
%                   floating integer ambiguity), 6 (3d fix, RTK with fixed
%                   integer ambiguity).
%               .hdop: 
%                   horizontal dilution of precision
%               .healthy: 
%                   whether the GNSS receiver is healthy. Unhealthy 
%                   is defined as missing 5 frames of data in a row at the 
%                   expected rate.
%               .horz_acc_m:
%                   estimated horizontal position accuracy, m
%               .lat_rad:
%                   latitude, rad
%               .latency_ms:
%                   latency between the time of GPS signal and recording
%               .lon_rad:
%                   longitude, rad
%               .ned_vel_mps:
%                   ned_vel_mps
%               .new_data:
%                   whether new data was received by the GNSS receiver
%               .num_sats:
%                   the number of satellites used in the GNSS solution
%               .pps_timestamp_us:
%                   pulse per second timestamp in us
%               .spd_mps:
%                   ground speed, m/s (2 norm of ned_vel_mps)
%               .systime_towfullSecond_us:
%                   system time of week to the full second in us
%               .tow_ms:
%                   GNSS time of week, ms
%               .track_acc_rad:
%                   ground track, rad
%               .vdop:
%                   vertical dilution of precision
%               .vel_acc_mps
%                   estimated velocity accuracy, m/s
%               .vert_acc_m:
%                   estimated vertical position accuracy, m
%               .week
%                   GNSS week number
%           .imu - data from imu
%               .accel_mps2:
%                   the accelerometer data, with bias and scale factor 
%                   corrected, and rotated into the vehicle frame, m/s/s 
%                   [x y z]
%               .die_temp_c:
%                    the IMU die temperature, C
%               .gyro_radps:
%                   the gyro data, with bias corrected, and rotated into 
%                   the vehicle frame, rad/s [x y z]
%               .healthy: 
%                   whether the accelerometer and gyro are healthy. 
%                   Unhealthy is defined as missing 5 frames of data in a 
%                   row at the expected rate.
%               .new_data:
%                   whether new data was received from the accelerometer 
%                   and gyro
%           .mag - data from the magnetometer
%               .healthy:
%                   whether the magnetometer is healthy. Unhealthy is 
%                   defined as missing 5 frames of data in a row at the 
%                   expected rate
%               .ut:
%                   the magnetometer data, with bias and scale factor 
%                   corrected, and rotated into the vehicle frame, uT 
%                   [x y z]
%               .new_data:
%                   whether new data was received from the magnetometer
%           .incept - Inceptor Data
%               .ch:  
%                   SBUS channel values. SBUS is 11 bits with a range of 
%                   0 - 2048. Some SBUS receivers, such as FrSky, use a 
%                   default range of 172 - 1811, unless an extended range 
%                   is configured
%               .ch17: 
%                   some SBUS transmitters and receivers support two 
%                   boolean outputs, CH 17 and CH 18, which are available 
%                   here
%               .ch18: 
%                   some SBUS transmitters and receivers support two 
%                   boolean outputs, CH 17 and CH 18, which are available 
%                   here
%               .failsafe: 
%                   whether the SBUS receiver has entered failsafe mode - 
%                   this typically occurs if many frames of data are lost 
%                   in a row
%               .lost_frame:  
%                   whether a frame of SBUS data was lost by the receiver
%               .new_data: 
%                   whether new data was received by the SBUS receiver
%           .nav - navigation filter data
%               .accel - accelerometer data
%                   .bias_mps2:
%                       gyro bias estimate from the EKF, rad/s [x y z]
%                   .mps2:
%                       IMU acceleterometer data with the EKF estimated 
%                       biases removed and digital low pass filtereing 
%                       applied, m/s/s [x y z]
%               .alt - altitude data
%                   .msl_m: 
%                       altitude above Mean Sea Level (MSL), m
%                   .pres_m: 
%                       pressure altitude, m
%                   .rel_m: 
%                       altitude above where the navigation filter was 
%                       initialized, m
%                   .wgs84_m: 
%                       altitude above the WGS84 ellipsoid, m
%               .gnd - ground data
%                   .spd_mps: 
%                       ground speed, m/s
%                   .track_rad: 
%                       ground track, rad
%               .gyro - gyro data
%                   .bias_radps: 
%                       gyro bias estimate from the EKF, rad/s [x y z]
%                   .radps: 
%                       IMU gyro data with the EKF estimated biases removed
%                       and digital low pass filtereing applied, rad/s 
%                       [x y z]
%               .home
%                   .alt_wgs84_m:
%                       home location (i.e. origin of the NED position) 
%                       above the WGS84 ellipsoid, m
%                   .lat_rad: 
%                       home location (i.e. origin of the NED position) 
%                       latitude, rad
%                   .lon_rad: 
%                       home location (i.e. origin of the NED position) 
%                       longitude, rad
%               .initialized:
%                   whether the navigation filter has been initialized. Do 
%                   not use navigation filter data before it has been 
%                   initialized. Requires a good GNSS solution to complete 
%                   the initialization process.
%               .lat_rad:
%                   latitude, rad
%               .lon_rad:
%                   latitude, rad
%               .mag_ut:
%                   IMU magnetometer data with digital low pass filtering 
%                   applied, uT [x y z]
%               .ned
%                   .pos_m: 
%                       North east down position relative to where the 
%                       navigation filter was initialized, m 
%                       [north east down]
%                   .vel_mps: 
%                       North east down ground velocity, m/s 
%                       [north east down]
%               .flight_path_rad:
%                       flight path angle, rad
%               .heading_rad:
%                       heading angle relative to true north, rad                       
%               .pitch_rad:
%                       pitch angle, rad
%               .roll_rad:
%                       roll angle, rad
%               .ias_mps:
%                       indicated airspeed, m/s
%               .diff_press_pa:
%                       filtered differential pressure, Pa
%               .static_press_pa:
%                       filtered static pressure, Pa
%           .pres
%               .pitot_static_installed
%               .diff
%                   .die_temp_c:
%                       the pressure transducer die temperature, C
%                   .healthy:
%                       whether the pressure transducer is healthy. 
%                       Unhealthy is defined as missing 5 frames of data in
%                       a row at the expected rate.
%                   .new_data:
%                       whether new data was received from the pressure 
%                       transducer
%                   .pres_pa:
%                       the measured pressure, Pa
%               .static
%                   .die_temp_c:
%                       the pressure transducer die temperature, C
%                   .healthy:
%                       whether the pressure transducer is healthy. 
%                       Unhealthy is defined as missing 5 frames of data in
%                       a row at the expected rate.
%                   .new_data:
%                       whether new data was received from the pressure 
%                       transducer
%                   .pres_pa:
%                       the measured pressure, Pa
%           .pwr
%               .mod_curr_v:
%                   voltage measured on the power port current pin. 
%                   Typically this is scaled by the power module mA / volt 
%                   value and is power module specific
%               .mod_volt_v
%                   voltage measured on the power port voltage pin. Note 
%                   that this is not the battery pack voltage, typically 
%                   this value needs to be scaled by the power module 
%                   volts / volt value and is power module specific
%               .rpm:
%                   revolutions per second of the motor
%           .sys - system data
%               .telem_param:
%                   n array of in-flight-tunable parameters sent from the 
%                   ground station. NUM_TELEM_PARAMS defines the number of 
%                   parameters available, typically 24. These parameters 
%                   can be used for anything that might be adjusted in 
%                   flight, such as controlling gains, selecting excitation
%                   waveforms, etc.
%               .frame_time_us:
%                   time the previous frame took to complete, us. Useful 
%                   for analyzing CPU load.
%               .time_gnss_s
%                   time since boot with syncronized GNSS time.
%               .time_s:
%                   time since boot, s
%           .vms - vms data
%               .analog: 
%                   ADC voltages converted to engineering units (i.e. POT 
%                   voltage to control surface deflection)
%               .aux:
%                   aux variables - these are undefined and can be used by 
%                   the developer to output data for logging. Useful for 
%                   logging internal control law states, research 
%                   variables, or other values of interest. NUM_AUX_VAR 
%                   defines the number of channels available, currently 24.
%               .batt_consumed_mah:
%                   battery pack capacity consumed, mAh
%               .batt_curr_ma:
%                   battery pack current draw, mA
%               .batt_remaining_prcnt: 
%                   battery pack capacity remaining, %
%               .batt_remaining_time_s:
%                   estimated flight time remaining, s
%               .batt_volt_v: 
%                   battery pack voltage
%               .IAS_mps:
%                   indicated airspeed, m/s
%               .mode:  
%                   the current aircraft mode - 0 (manual flight mode),
%                   (1) stability augmented flight mode, (2) attitude 
%                   feedback flight mode, (3) autonomous flight mode, (4) 
%                   test point / research flight mode.
%               .motors_enabled: 
%                   whether the motors are enabled and can turn. This is 
%                   not a command, rather just feedback provided from the 
%                   VMS about whether the motors are "hot" and is used in 
%                   telemetry and for operator situation awareness.
%               .pwm_cmd: 
%                   angle or PLA commands to PWM channels. This is used to 
%                   drive the simulation.
%               .pwm_cnt: 
%                   raw PWM counts to PWM channels. This is sent to the 
%                   aircraft effectors. Typically a polynomial evaluation 
%                   would be used to convert from an angle command (i.e. an
%                   aileron deflection) to raw PWM output.
%               .sbus_ch17: 
%                   output command to SBUS CH 17.
%               .sbus_ch18: 
%                   output command to SBUS CH 18.
%               .sbus_cnt: 
%                   raw SBUS counts to SBUS channels. This is sent to the 
%                   aircraft effectors. Typically a polynomial evaluation 
%                   would be used to convert from an angle command (i.e. an
%                   aileron deflection) to raw SBUS output.
%               .throttle_cmd_prcnt: 
%                   the throttle command given as a %, this is used for 
%                   telemetry and situational awareness.
%               .waypoint_reached: 
%                   whether the current waypoint has been 
%                   reached. This is used to indicate to the ground station
%                   that the active waypoint should be advanced to the next
%                   in the flight plan.
%           .waypoint - waypoint data 
%               .cmd:
%                   the command associated with the MissionItem
%               .frame:
%                   the coordinate frame of the MissionItem
%               .param1:
%                   command dependent parameter
%               .param2: 
%                   command dependent parameter
%               .param3: 
%                   command dependent parameter
%               .param4: 
%                   command dependent parameter
%               .x: 
%                   typically latitude represented as 1e7 degrees
%               .y: 
%                   typically longitude represented as 1e7 degrees
%               .z: 
%                   typically altitude, but can be dependent on the command
%                   and frame
%       .n580 - data from the Honeywell n580
%           .raw_headers: 
%               string of raw data column headers
%           .raw 
%               matrix of raw data (time, attitude, accel, gyro)
%           .filt_headers: 
%               string of filter data column headers
%           .filt: 
%               matrix of time, position, velocity, solution of filtered 
%               data
%       .data_FMUR - data structure from process_FMUR_data.fcn
%                       will return an empty data structure if crop and
%                       process is false
%       .data_n580 - data structure from process_n580_data.fcn
%                       will return an empty data structure if crop and
%                       process is false
%       .data_int - integrated data structure from
%                       combine_n580_FMUR_data.fcn
%           
% 
% Sam Jaeger
% jaege246@umn.edu
% 10/26/2025
%   Revised: 7/1/2026

function test_data = read_flight_test_data  

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FMU-R
disp('Select .mat datafile from the FMU-R')
[FMUR_file,FMUR_floc] = uigetfile('*.mat');
load(append(FMUR_floc,'\',FMUR_file));

% potemometer data --------------------------------------------------------
test_data.FMUR.adc_volt = adc_volt;


% gnss data ---------------------------------------------------------------
test_data.FMUR.gnss.alt_msl_m = gnss_alt_msl_m;
test_data.FMUR.gnss.alt_wgs84_m = gnss_alt_wgs84_m;
test_data.FMUR.gnss.fix = gnss_fix;
test_data.FMUR.gnss.hdop = gnss_hdop;
test_data.FMUR.gnss.healthy = gnss_healthy;
test_data.FMUR.gnss.horz_acc_m = gnss_horz_acc_m;
test_data.FMUR.gnss.lat_rad = gnss_lat_rad;
test_data.FMUR.gnss.latency_ms = gnss_latency_ms;
test_data.FMUR.gnss.lon_rad = gnss_lon_rad;
test_data.FMUR.gnss.ned_vel_mps = gnss_ned_vel_mps;
test_data.FMUR.gnss.new_data = gnss_new_data;
test_data.FMUR.gnss.num_sats = gnss_num_sats;
test_data.FMUR.gnss.pps_timestamp_us = gnss_pps_timestamp_us;
test_data.FMUR.gnss.spd_mps = gnss_spd_mps;
test_data.FMUR.gnss.systime_towfullSecond_us = gnss_systime_towfullSecond_us;
test_data.FMUR.gnss.tow_ms = gnss_tow_ms;
test_data.FMUR.gnss.track_acc_rad = gnss_track_acc_rad;
test_data.FMUR.gnss.track_rad = gnss_track_rad;
test_data.FMUR.gnss.vdop = gnss_vdop;
test_data.FMUR.gnss.vel_acc_mps = gnss_vel_acc_mps;
test_data.FMUR.gnss.vert_acc_m = gnss_vert_acc_m;
test_data.FMUR.gnss.week = gnss_week;


% IMU Data ----------------------------------------------------------------
test_data.FMUR.imu.accel_mps2 = imu_accel_mps2;
test_data.FMUR.imu.die_temp_c = imu_die_temp_c;
test_data.FMUR.imu.gyro_radps = imu_gyro_radps;
test_data.FMUR.imu.healthy = imu_healthy;
test_data.FMUR.imu.new_data = imu_new_data;


% Magnetometer Data -------------------------------------------------------
test_data.FMUR.mag.healthy = imu_mag_healthy;
test_data.FMUR.mag.ut = imu_mag_ut;
test_data.FMUR.mag.new_data = imu_new_mag_data;


% Incept data -------------------------------------------------------------
test_data.FMUR.incept.ch = incept_ch;
test_data.FMUR.incept.ch17 = incept_ch17;
test_data.FMUR.incept.ch18 = incept_ch18;
test_data.FMUR.incept.failsafe = incept_failsafe;
test_data.FMUR.incept.lost_frame = incept_lost_frame;
test_data.FMUR.incept.new_data = incept_new_data;


% nav data (sensor fusion) ------------------------------------------------
test_data.FMUR.nav.accel.bias_mps2 = nav_accel_bias_mps2;
test_data.FMUR.nav.accel.mps2 = nav_accel_mps2;

test_data.FMUR.nav.alt.msl_m = nav_alt_msl_m;
test_data.FMUR.nav.alt.pres_m = nav_alt_pres_m;
test_data.FMUR.nav.alt.rel_m = nav_alt_rel_m;
test_data.FMUR.nav.alt.wgs84_m = nav_alt_wgs84_m;

test_data.FMUR.nav.gnd.spd_mps = nav_gnd_spd_mps;
test_data.FMUR.nav.gnd.track_rad = nav_gnd_track_rad;

test_data.FMUR.nav.gyro.bias_radps = nav_gyro_bias_radps;
test_data.FMUR.nav.gyro.radps = nav_gyro_radps;

test_data.FMUR.nav.home.alt_wgs84_m = nav_home_alt_wgs84_m;
test_data.FMUR.nav.home.lat_rad = nav_home_lat_rad;
test_data.FMUR.nav.home.lon_rad = nav_home_lon_rad;

test_data.FMUR.nav.initialized = nav_initialized;
test_data.FMUR.nav.lat_rad = nav_lat_rad;
test_data.FMUR.nav.lon_rad = nav_lon_rad;

test_data.FMUR.nav.mag_ut = nav_mag_ut;

test_data.FMUR.nav.ned.pos_m = nav_ned_pos_m;
test_data.FMUR.nav.ned.vel_mps = nav_ned_vel_mps;

test_data.FMUR.nav.flight_path_rad = nav_flight_path_rad;
test_data.FMUR.nav.heading_rad = nav_heading_rad;
test_data.FMUR.nav.pitch_rad = nav_pitch_rad;
test_data.FMUR.nav.roll_rad = nav_roll_rad;

test_data.FMUR.nav.ias_mps = nav_ias_mps;
test_data.FMUR.nav.diff_pres_pa = nav_diff_pres_pa;
test_data.FMUR.nav.static_pres_pa = nav_static_pres_pa;


% pitot static pressure sensing -------------------------------------------
test_data.FMUR.pres.pitot_static_installed = pitot_static_installed;
test_data.FMUR.pres.diff.die_temp_c = pres_diff_die_temp_c;
test_data.FMUR.pres.diff.healthy = pres_diff_healthy;
test_data.FMUR.pres.diff.new_data = pres_diff_new_data;
test_data.FMUR.pres.diff.pres_pa = pres_diff_pres_pa;
test_data.FMUR.pres.static.die_temp_c = pres_static_die_temp_c;
test_data.FMUR.pres.static.healthy = pres_static_healthy;
test_data.FMUR.pres.static.new_data = pres_static_new_data;
test_data.FMUR.pres.static.pres_pa = pres_static_pres_pa;


% power data --------------------------------------------------------------
test_data.FMUR.pwr.mod_curr_v = pwr_mod_curr_v;
test_data.FMUR.pwr.mod_volt_v = pwr_mod_volt_v;
test_data.FMUR.pwr.rpm = rpm;

% system data (timing) ----------------------------------------------------
test_data.FMUR.sys.telem_param = telem_param;
test_data.FMUR.sys.frame_time_us = sys_frame_time_us;
test_data.FMUR.sys.time_gnss_s = sys_time_gnss_s;
test_data.FMUR.sys.time_s = sys_time_s;


% vms data ----------------------------------------------------------------
test_data.FMUR.vms.analog = vms_analog;
test_data.FMUR.vms.aux = vms_aux;
test_data.FMUR.vms.batt_consumed_mah = vms_batt_consumed_mah;
test_data.FMUR.vms.batt_curr_ma = vms_batt_curr_ma;
test_data.FMUR.vms.batt_remaining_prcnt = vms_batt_remaining_prcnt;
test_data.FMUR.vms.batt_remaining_time_s = vms_batt_remaining_time_s;
test_data.FMUR.vms.batt_volt_v = vms_batt_volt_v;
test_data.FMUR.vms.IAS_mps = vms_IAS_mps;
test_data.FMUR.vms.mode = vms_mode;
test_data.FMUR.vms.motors_enabled = vms_motors_enabled;
test_data.FMUR.vms.pwm_cmd = vms_pwm_cmd;
test_data.FMUR.vms.pwm_cnt = vms_pwm_cnt;
test_data.FMUR.vms.sbus_ch17 = vms_sbus_ch17;
test_data.FMUR.vms.sbus_ch18 = vms_sbus_ch18;
test_data.FMUR.vms.sbus_cmd = vms_sbus_cmd;
test_data.FMUR.vms.sbus_cnt = vms_sbus_cnt;
test_data.FMUR.vms.throttle_cmd_prcnt = vms_throttle_cmd_prcnt;
test_data.FMUR.vms.waypoint_reached = vms_waypoint_reached;


% waypoint data -----------------------------------------------------------
test_data.FMUR.waypoint.cmd = waypoint_cmd;
test_data.FMUR.waypoint.frame = waypoint_frame;
test_data.FMUR.waypoint.param1 = waypoint_param1;
test_data.FMUR.waypoint.param2 = waypoint_param2;
test_data.FMUR.waypoint.param3 = waypoint_param3;
test_data.FMUR.waypoint.param4 = waypoint_param4;
test_data.FMUR.waypoint.x = waypoint_x;
test_data.FMUR.waypoint.y = waypoint_y;
test_data.FMUR.waypoint.z = waypoint_z;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% n580
disp('n580 Data Recorded?')
n580tf = input('true/false:  ');

if n580tf == true
    disp('Select .csv datafile for the n580 attitude solution.')
    [n580_Attitude_file,n580_Attitude_floc] = uigetfile('*.csv');
    Attitude = readmatrix(append(n580_Attitude_floc,'\',n580_Attitude_file));


    disp('Select .csv datafile for the n580 nav solution (raw imu data).')
    [n580_nav_file,n580_nav_floc] = uigetfile('*.csv');
    Nav = readmatrix(append(n580_nav_floc,'\',n580_nav_file));

    disp('Select .csv datafile for the n580 position solution.')
    [n580_pos_file,n580_pos_floc] = uigetfile('*.csv');
    Position = readmatrix(append(n580_pos_floc,'\',n580_pos_file));

    disp('Select .csv datafile for the n580 velocity solution.')
    [n580_vel_file,n580_vel_floc] = uigetfile('*.csv');
    Velocity = readmatrix(append(n580_vel_floc,'\',n580_vel_file));


    % Assemble into raw and filtered data structures, remove message column
    test_data.n580.raw_headers = ["SystemTOV",... 
                                   "GPSTOV",...
                                   "GPSMode",...
                                   "INSMode",...
                                   "TrueHeading_radians",...
                                   "Pitch_radians",...
                                   "Roll_radians",...
                                   "DeltaAngleX_radians",...
                                   "DeltaAngleY_radians",...
                                   "DeltaAngleZ_radians",...
                                   "DeltaVelocityX_mps",...
                                   "DeltaVelocityY_mps",...
                                   "DeltaVelocityZ_mps"];
    test_data.n580.raw = [Attitude(:,2:end), Nav(:,2:end)];

    test_data.n580.filt_headers = ["SystemTOV",...
                                       "GPSTOV",...
                                       "GPSMode",...
                                       "INSMode",...
                                       "Easting",...
                                       "Northing",...
                                       "Latitude_rad",...
                                       "Longitude_rad",...
                                       "Altitude_m",...
                                       "LatitudeStdv",...
                                       "LongitudeStdv",...
                                       "AltitudeStdv",...
                                       "VelocityX_mps",...
                                       "VelocityY_mps",...
                                       "VelocityZ_mps"];
    test_data.n580.filt = [Position(:,2:end), Velocity(:,6:end)];
else
    test_data.n580 = [];
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% process flight test data
disp('Crop and process FMU-R data?')
crop_process_ftd = input('true / false:    ');

if crop_process_ftd == true
    disp('Select .mat data file for calibration.')
    [cal_file,cal_floc] = uigetfile('*.mat');
    load(append(cal_floc,'\',cal_file),'cal');
    
    disp('Input altimeter setting and temperature of the day.')
    altimeter =   input('altimeter (in Hg):    ');
    temperature = input('temperature (F):      ');

    [t_GNSS_se,i_FMUR_se, i_n580_raw_se, i_n580_filt_se, ~] = crop_flight_test_data(test_data);
    test_data.data_FMUR = process_FMUR_data(test_data, i_FMUR_se, cal, altimeter, temperature);
    
    if n580tf == true
        test_data.data_n580 = process_n580_data(test_data, t_GNSS_se(1), t_GNSS_se(2), i_n580_raw_se, i_n580_filt_se);
        test_data.data_int = combine_n580_FMUR_data(test_data.data_FMUR, test_data.data_n580);

        flight_test_plots(test_data.data_int)
    else
        test_data.data_n580 = [];
        test_data.data_int = [];
    end
else
    test_data.data_FMUR = [];
    test_data.data_n580 = [];
    test_data.data_int = [];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% save test data
    disp('Save flight test data?')
    save_ftd = input('true / false:   ');
    if save_ftd == true
        ftd_fname = input('filename: ','s');
        save(string(ftd_fname),'test_data')
    end
end