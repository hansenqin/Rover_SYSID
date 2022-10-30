function [time, x, y, h, u, v, r, udot, vdot, rdot, delta_cmd] = load_states(states)
% Loads in the states and other data for use in other functions for
% more concise code
time = states.time;
x = states.x;
y = states.y;
h = states.h;
delta_cmd = states.delta_cmd;
u = states.u;
v = states.v;
r = states.r;
udot = states.udot;
vdot = states.vdot;
rdot = states.rdot;
end