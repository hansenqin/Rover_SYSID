function syms_derivative_equation = bezier_derivative_syms(syms_equation, time_range)
   % Takes in a symbolic equation and returns the symbolic
   % equation for its derivative. Note that for syms_equation
   % and syms_derivative_equation, the inputs to these
   % functions must be normalized
   syms t;
   syms_derivative_equation = diff(syms_equation, t)/time_range;
end