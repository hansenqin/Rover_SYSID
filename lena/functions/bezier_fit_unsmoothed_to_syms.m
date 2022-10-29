function [syms_equation] = bezier_fit_unsmoothed_to_syms(time, unsmoothed_variable)
   % Takes in unsmoothed x, y, h, or w, and a normalized time. Returns a MATLAB
   % function that can be used to find the smoothed values at
   % some time using syms_equation(time). NOTE: time must be
   % normalized aka [0,1]

   % define the first point
   unsmoothed_variable_0 = unsmoothed_variable(1);

   % Define the last point (meaning last point of the time array)
   unsmoothed_variable_1 = unsmoothed_variable(end);

   % Solve
   b = [unsmoothed_variable_0; unsmoothed_variable_1];
   syms t;
   A_1 = @(t)[(1-t).^5; 5.*t.*(1-t).^4; 10.*(t.^2).*(1-t).^3; 10.*(t.^3).*(1-t).^2; 5.*(t.^4).*(1-t).^1; t.^5];
   A = [A_1(0), A_1(1)]';
   
   x0 = [1, 1, 1, 1, 1, 1];
   A_points = A_1(time);
   f = @(x) norm(A_points'*[x(1); x(2); x(3); x(4); x(5); x(6)]-unsmoothed_variable');
   P=fmincon(f, x0, [], [], A, b);
   % Return the equation
   syms_equation = sym(A_1)'*P';
end