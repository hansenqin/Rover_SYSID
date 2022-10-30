function [lf, lr, m, Izz, servo_offset, rw, g, robot_frame] = load_rover_config_constants(rover_config)
% Loads in the constants of rover_config for more concise use in other
% functions
    lf = rover_config.lf;
    lr = rover_config.lr;
    m = rover_config.m;
    Izz = rover_config.Izz;
    servo_offset = rover_config.servo_offset;
    rw = rover_config.rw;
    g = rover_config.g;
    robot_frame = rover_config.robot_frame;
end