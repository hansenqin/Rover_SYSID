function [x, y, h, u, v, r, udot, vdot, rdot, delta_cmd, xdot, ydot] = bezier_fit_to_data(time, x, y, h, delta_cmd)
       % Uses bezier_fit_unsmoothed_to_syms, 
       % syms_derivative_equation_syms, and bezier_rotate_syms. 
       % Takes in unsmoothed x, y, h, and w data and sets the 
       % obj.trajectories x, y, h, u, v, r, udot, vdot, rdot, and 
       % w to their smoothed values. 
       if ~isempty(time)
       
       % Normalize the time
       plot(time(1:end-1), diff(x).*cos(h(1:end-1))./diff(time) + diff(y)./diff(time).*sin(h(1:end-1)));
       time_range = time(end)-time(1);
       time_normal = (time-time(1))./time_range;

       % Smooth the data
       syms_x = bezier_fit_unsmoothed_to_syms(time_normal, x);
       syms_y = bezier_fit_unsmoothed_to_syms(time_normal, y);
       syms_h = bezier_fit_unsmoothed_to_syms(time_normal, h);


        %% Polar Calculations
        theta = atan2(y,x);
        r = sqrt(x.^2 + y.^2);

        syms_theta = bezier_fit_unsmoothed_to_syms(time_normal, theta);
        syms_r = bezier_fit_unsmoothed_to_syms(time_normal, r);

        % Note that theta dot is the yaw rate
        syms_theta_dot = bezier_derivative_syms(syms_theta, time_range);
        syms_theta_dot_dot = bezier_derivative_syms(syms_theta_dot, time_range);

        func_theta_polar = matlabFunction(syms_theta);
       func_theta_dot_polar = matlabFunction(syms_theta_dot);
       func_theta_dot_dot_polar = matlabFunction(syms_theta_dot_dot);
       
       theta_polar = func_theta_polar(time_normal);
       theta_dot_polar = func_theta_dot_polar(time_normal);
       theta_dot_dot_polar = func_theta_dot_dot_polar(time_normal);

        % Can get x and y from fitted r and theta
        syms_x_polar = syms_r * cos(syms_theta);
        syms_y_polar = syms_r * sin(syms_theta);

                        func_x_polar = matlabFunction(syms_x_polar);
       func_y_polar = matlabFunction(syms_y_polar);

        x_polar = func_x_polar(time_normal);
        y_polar = func_y_polar(time_normal);

        

       syms_xdot_polar = bezier_derivative_syms(syms_x_polar,time_range);
       syms_ydot_polar = bezier_derivative_syms(syms_y_polar,time_range);

       func_xdot_polar = matlabFunction(syms_xdot_polar);
       xdot_polar = func_xdot_polar(time_normal);

       [syms_u_polar, syms_v_polar, ~] = bezier_rotate_syms(syms_xdot_polar, syms_ydot_polar, syms_h, syms_theta);
       func_u_polar = matlabFunction(syms_u_polar);
       func_v_polar = matlabFunction(syms_v_polar);

        u_polar = func_u_polar(time_normal);
       v_polar = func_v_polar(time_normal);

       syms_delta_cmd = bezier_fit_unsmoothed_to_syms(time_normal, delta_cmd);
       
       % Calculate the derivatives
       syms_xdot = bezier_derivative_syms(syms_x, time_range);
       syms_ydot = bezier_derivative_syms(syms_y, time_range);
       syms_hdot = bezier_derivative_syms(syms_h, time_range);

       % Calculate u, v, and r
       [syms_u, syms_v, syms_r] = bezier_rotate_syms(syms_xdot, syms_ydot, syms_hdot, syms_h);

       % Calculate the derivatives
       syms_udot = bezier_derivative_syms(syms_u, time_range);
       syms_vdot = bezier_derivative_syms(syms_v, time_range);
       syms_rdot = bezier_derivative_syms(syms_r, time_range);

       % Convert the syms equations to MATLAB equations
       func_x = matlabFunction(syms_x);
       func_y = matlabFunction(syms_y);
       func_h = matlabFunction(syms_h);
       func_u = matlabFunction(syms_u);
       func_v = matlabFunction(syms_v);
       func_r = matlabFunction(syms_r);
       func_udot = matlabFunction(syms_udot);
       func_vdot = matlabFunction(syms_vdot);
       func_rdot = matlabFunction(syms_rdot);
       func_delta_cmd = matlabFunction(syms_delta_cmd);

       func_xdot = matlabFunction(syms_xdot);
       func_ydot = matlabFunction(syms_ydot);

       % Calculate the values
       x = func_x(time_normal);
       y = func_y(time_normal);
       h = func_h(time_normal);
       u = func_u(time_normal);
       v = func_v(time_normal);
       r = func_r(time_normal);
       udot = func_udot(time_normal);
       vdot = func_vdot(time_normal);
       rdot = func_rdot(time_normal);
       delta_cmd = func_delta_cmd(time_normal);
       
       xdot = func_xdot(time_normal);
       ydot = func_ydot(time_normal);
       hold on
       plot(time, u)
       plot(time, u_polar)
       legend('original', 'bezier','polar')
       hold off
       end
       

        % h, r, and rdot almost linear
        % h = h;
%         r = diff(h)./diff(time); r(end+1) = r(end);
%         rdot = diff(r)./diff(time); rdot(end+1) = rdot(end);


   end