function [target_struct] = compile_position_state_vector(reference_struct, structs_to_sync, target_struct)
    % Inserts time, x, y, h, and delta_cmd for position state structs.
    % The reference struct provdes time, x ,y, and h while the
    % stucts_to_sync provides the delta_cmd. The target struct is returned
    % for use elsewhere
    target_struct.time = reference_struct.time;
    target_struct.x = reference_struct.x;
    target_struct.y = reference_struct.y;
    target_struct.h = reference_struct.h;
    target_struct.delta_cmd = structs_to_sync.rover_debug_states_out.delta_cmd;
end