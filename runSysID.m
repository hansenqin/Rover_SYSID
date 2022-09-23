sysID = tire_curve_sysID_helper_class('velodyne_sys_id_manual_driving.bag', 'velodyne_rover.json', 0);
[xLinearCoef, fLinearCoef, rLinearCoef, lambda, alphaf, alphar, F_x, F_yf, F_yr] = sysID.get_tire_curve_coefficients();
sysID.plot_linear_tire_curve(xLinearCoef, fLinearCoef, rLinearCoef, lambda, alphaf, alphar, F_x, F_yf, F_yr);