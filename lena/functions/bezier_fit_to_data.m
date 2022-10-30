function [x, y, h, u, v, r, udot, vdot, rdot, delta_cmd, xdot, ydot] = bezier_fit_to_data(time, x, y, h, delta_cmd)
       % Uses bezier_fit_unsmoothed_to_syms, 
       % syms_derivative_equation_syms, and bezier_rotate_syms. 
       % Takes in unsmoothed x, y, h, and w data and sets the 
       % obj.trajectories x, y, h, u, v, r, udot, vdot, rdot, and 
       % w to their smoothed values. 
       
       % Normalize the time
       time_range = time(end)-time(1);
       time_normal = (time-time(1))./time_range;

       % Smooth the data
       syms_x = bezier_fit_unsmoothed_to_syms(time_normal, x);
       syms_y = bezier_fit_unsmoothed_to_syms(time_normal, y);
       syms_h = bezier_fit_unsmoothed_to_syms(time_normal, h);
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

        % h, r, and rdot almost linear
        % h = h;
%         r = diff(h)./diff(time); r(end+1) = r(end);
%         rdot = diff(r)./diff(time); rdot(end+1) = rdot(end);


   end