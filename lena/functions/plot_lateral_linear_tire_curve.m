function plot_lateral_linear_tire_curve(fLinearCoef, rLinearCoef, alphaf, alphar, F_yf, F_yr)
   % Plots the linear lateral curves
   plot_single_lateral_linear_tire_curve(fLinearCoef, alphaf, F_yf, 1);
   plot_single_lateral_linear_tire_curve(rLinearCoef, alphar, F_yr, 0);
end

function plot_single_lateral_linear_tire_curve(yLinearCoef, alpha, F_y, position)
   % Plots the front or rear lateral curve depending on the
   % arguments. Note that for for front, position==1 and for rear,
   % position==0.
   if position % Front tire
       figure(2);
       clf;
       xlabel("Front Slip Angle");
       ylabel("Front Lateral Tire Force (N)");
       title("Fywf="+round(yLinearCoef(1),2)+"*alpha_f")
   else % Rear tire
       figure(3);
       clf;
       xlabel("Rear Slip Angle");
       ylabel("Rear Lateral Tire Force (N)");
       title("Fywr="+round(yLinearCoef(1),2)+"*alpha_r");
   end
   % Plot the data and the fitted curve
   scatter(alpha, F_y);
   hold on;
   yLinear = @(t) yLinearCoef(1)*t;
   t = -0.2:0.01:0.2;
   plot(t, yLinear(t), "LineWidth", 2);
   xlim([-0.3,0.3]);
   hold off;
end