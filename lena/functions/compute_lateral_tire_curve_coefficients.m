function [fLinearCoef, rLinearCoef, alphaf, alphar, F_yf, F_yr] = ...
    compute_lateral_tire_curve_coefficients(rover_config, fitted_states)
    % Calculate the lateral tire coefficients for the front and rear tires.
    % Takes in a struct of rover_config (from the json file) and a struct
    % of the states (x, y, h, u, v, r, udot, vdot, rdot)

    [lf, lr, m, Izz, servo_offset, rw, g, robot_frame] = load_rover_config_constants(rover_config);
    [time, x, y, h, u, v, r, udot, vdot, rdot, delta_cmd] = load_states(fitted_states);

    % Find slip angles
    alphaf = delta_cmd - (v + lf.*r)./sqrt(u.^2+0.05);
    alphar = -(v-lr.*r)./sqrt(u.^2+0.05);
    
    % Find lateral force
    F_yf = (lr.*m.*(vdot+u.*r)+rdot.*Izz)./(lf + lr);
    F_yr=(m.*(vdot+u.*r)-rdot*Izz./lf)./(1+lr./lf);

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