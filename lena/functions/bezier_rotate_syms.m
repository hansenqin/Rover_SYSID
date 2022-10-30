function [syms_u, syms_v, syms_r] = bezier_rotate_syms(syms_xdot, syms_ydot, syms_hdot, syms_h)
   % Takes in symbolic equations for xdot, ydot, and hdot.
   % Returns symbolic equations for u, v, and r. Note that for
   % all of the symbolic equations, the inputs to these
   % functions must be normalized
   
   % Solve for equations
   syms_u = syms_xdot.*cos(syms_h)+ syms_ydot.*sin(syms_h);
   syms_v = -syms_xdot.*sin(syms_h)+ syms_ydot.*cos(syms_h);
   syms_r = syms_hdot;

end