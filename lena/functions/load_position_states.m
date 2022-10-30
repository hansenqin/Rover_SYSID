function [time, x, y, h, delta_cmd] = load_position_states(states)
% Loads in the states and other data for use in other functions for
% more concise code
time = states.time;
x = states.x;
y = states.y;
h = states.h;
delta_cmd = states.delta_cmd;
end