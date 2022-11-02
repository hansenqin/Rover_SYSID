function [fLinearCoef, rLinearCoef, alphaf, alphar, F_yf, F_yr] = ...
    compute_lateral_tire_curve_coefficients(rover_config, fitted_states)
    % Calculate the lateral tire coefficients for the front and rear tires.
    % Takes in a struct of rover_config (from the json file) and a struct
    % of the states (x, y, h, u, v, r, udot, vdot, rdot)

    [lf, lr, m, Izz, servo_offset, rw, g, robot_frame] = load_rover_config_constants(rover_config);
    [time, x, y, h, u, v, r, udot, vdot, rdot, delta_cmd] = load_states(fitted_states);


%     time1 = time(1):0.02:time(end);
% x = spline(time,x,time1);
% y = spline(time,y,time1);
% h = spline(time,h,time1);
% u = spline(time,u,time1);
% v = spline(time,v,time1);
% r = spline(time,r,time1);
% udot = spline(time,udot,time1);
% vdot = spline(time,vdot,time1);
% rdot = spline(time,rdot,time1);
% delta_cmd = spline(time,delta_cmd,time1);

    
%     u  = smooth_using_cubic(time, u);
%     v  = smooth_using_cubic(time, v);
%     r  = smooth_using_cubic(time, r);
%     udot = smooth_using_cubic(time, udot);
%     vdot = smooth_using_cubic(time, vdot);
%     rdot = smooth_using_cubic(time, rdot);
    

    % Find slip angles
    alphaf = delta_cmd - (v + lf.*r)./sqrt(u.^2+0.05);
    alphar = -(v-lr.*r)./sqrt(u.^2+0.05);

    % Find lateral force
    F_yf = (lr.*m.*(vdot+u.*r)+rdot.*Izz)./(lf + lr);
    F_yr=(m.*(vdot+u.*r)-rdot*Izz./lf)./(1+lr./lf);

%     alphaf = smoothdata(alphaf);
%     alphar = smoothdata(alphar);
%     F_yf = smoothdata(F_yf);
%     F_yr = smoothdata(F_yr);

    alphar = alphar(abs(alphar)<0.3);
    F_yr = F_yr(abs(alphar)<0.3);
% condition = or(abs(alphar)<0.5, abs(F_yr)>0.2);
%     alphar = alphar(condition);
%     F_yr = F_yr(condition);
% 
% 
    alphaf = alphaf(abs(alphar)<0.3);
    F_yf = F_yf(abs(alphar)<0.3);
        alphaf = alphaf(abs(F_yf)<20);
    F_yf = F_yf(abs(F_yf)<20);
       condition = or(abs(alphaf)<0.1, abs(F_yf)>1);
    alphaf = alphaf(condition);
    F_yf = F_yf(condition);


    % Select and fit a curve to the data in the linear region 
    alphafSelected = alphaf(abs(alphaf)<0.2);
    alpharSelected = alphar(abs(alphar)<0.2);
    F_yfSelected = F_yf(abs(alphaf)<0.2);
    F_yrSelected = F_yr(abs(alphar)<0.2);

    % Lateral linear curve through (0,0) using fmincon
    x0 = 1;
    f = @(x) norm(alphafSelected*x-F_yfSelected);
    fLinearCoef = fmincon(f,x0);
    f = @(x) norm(alpharSelected*x-F_yrSelected);
    rLinearCoef = fmincon(f,x0);
            % Nonlinear curve using fmincon
%             x0 = [1 1];
%             f = @(x) norm(x(1)*tanh(x(2)*alphaf)-F_yf);
%             fNonlinearCoef = fmincon(f,x0);
%             r = @(x) norm(x(1)*tanh(x(2)*alphar)-F_yr);
%             rNonlinearCoef = fmincon(r,x0);
end

function var = smooth_using_cubic(time, var)
    p  = 1e-1;           % initialize smoothing constant
    fn = csaps(time, var, p); % get ppform of the cubic smoothing spline
    var = ppval(fn, time);   % evaluate piecewise polynomial
end