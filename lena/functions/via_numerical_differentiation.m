   function [time, x, y, h, u, v, r, udot, vdot, rdot, delta_cmd] = via_numerical_differentiation(structs)
        time = structs.vehicle_states_from_mocap.time;
        x = structs.vehicle_states_from_mocap.x;
        y = structs.vehicle_states_from_mocap.y;
        h = structs.vehicle_states_from_mocap.h;
        delta_cmd = structs.rover_debug_states_out.delta_cmd;

        xdot = diff(x)./diff(time);
        ydot = diff(y)./diff(time);
        hdot = diff(h)./diff(time);

        x = x(1:end-1);
        y = y(1:end-1);
        h = h(1:end-1);
        time = time(1:end-1);

        u = zeros(length(h), 1);
        v = zeros(length(h), 1);
        r = zeros(length(h), 1);

        for i=1:length(h)
        u(i) = norm(xdot(i).*cos(h(i)), ydot(i).*sin(h(i)));
        v(i) = norm(-xdot(i).*sin(h(i)), ydot(i).*cos(h(i)));
        r(i) = hdot(i);
        end

        udot = diff(u)./diff(time);
        vdot = diff(v)./diff(time);
        rdot = diff(r)./diff(time);

        x = x(1:end-1);
        y = y(1:end-1);
        h = h(1:end-1);
        u = u(1:end-1);
        v = v(1:end-1);
        r = r(1:end-1);
        time = time(1:end-1);
        delta_cmd = delta_cmd(1:end-2);
   end