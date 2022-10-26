% sysID = tire_curve_sysID_helper_class('sysid_09_27_straightline.bag', 'velodyne_rover.json', 0);
% [xLinearCoef, fLinearCoef, rLinearCoef, lambda, alphaf, alphar, F_x, F_yf, F_yr] = sysID.get_tire_curve_coefficients();
% sysID.plot_linear_tire_curve(xLinearCoef, fLinearCoef, rLinearCoef, lambda, alphaf, alphar, F_x, F_yf, F_yr);

%% Run Longitudnal Sysid
sysID = rover_longitudinal_sysid_class('mocap_longitudnal_10_09.bag', 'velodyne_rover.json', 0);
close all;

%%

[xLinearCoef, lambda, F_x] = sysID.get_longitudnal_tire_curve_coefficients();
sysID.plot_longitudnal_linear_tire_curve(xLinearCoef, lambda, F_x);


%% Run Lateral Sysid
clc
sysID = rover_lateral_sysid_class('mocap_lateral_10_09_1.bag', 'velodyne_rover.json', 0);
close all;