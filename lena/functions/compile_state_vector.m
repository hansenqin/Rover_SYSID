function [target_struct] = compile_state_vector(time, x, y, h, u, v, r, udot, vdot, rdot, delta_cmd, target_struct)
    % Inserts time, x, y, h, and delta_cmd for position state structs.
    % The reference struct provdes time, x ,y, and h while the
    % stucts_to_sync provides the delta_cmd. The target struct is returned
    % for use elsewhere
    target_struct.time = time;
    target_struct.x = x;
    target_struct.y = y;
    target_struct.h = h;
    target_struct.delta_cmd = delta_cmd;
    target_struct.u = u;
    target_struct.v = v;
    target_struct.r = r;
    target_struct.udot = udot;
    target_struct.vdot = vdot;
    target_struct.rdot = rdot;
end