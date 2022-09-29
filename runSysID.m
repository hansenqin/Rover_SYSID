% sysID = tire_curve_sysID_helper_class('sysid_09_27_straightline.bag', 'velodyne_rover.json', 0);
% [xLinearCoef, fLinearCoef, rLinearCoef, lambda, alphaf, alphar, F_x, F_yf, F_yr] = sysID.get_tire_curve_coefficients();
% sysID.plot_linear_tire_curve(xLinearCoef, fLinearCoef, rLinearCoef, lambda, alphaf, alphar, F_x, F_yf, F_yr);

%% Run Longitudnal Sysid
sysID = tire_curve_sysID_helper_class('sysid_09_27_straightline.bag', 'velodyne_rover.json', 0);
sysID.fit_bezier_to_trajectory();
sysID.filter_u_based_on_udot();
sysID.combine_all_trajectories();
[xLinearCoef, lambda, F_x] = sysID.get_longitudnal_tire_curve_coefficients();
sysID.plot_longitudnal_linear_tire_curve(xLinearCoef, lambda, F_x);