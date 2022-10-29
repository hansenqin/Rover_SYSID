function structs = remove_negative_u(reference_struct, structs, minimum_u)
   % Removes areas of negative/very small u for the given structs. Note
   % that the times of the structs do not have to match. The reference
   % struct must have time, x, y, and h

   % Find where the reference struct has negative/small u. This is done via
   % numerical differentiation, which although is not the most accurate,
   % will give an acceptable approximation
   xdot = (diff(reference_struct.x)./diff(reference_struct.time))';
   ydot = (diff(reference_struct.y)./diff(reference_struct.time))';
   h = zeros(1,length(reference_struct.h)-1);
   for i=1:length(h)
       h(i) = (reference_struct.h(i)+reference_struct.h(i+1))/2;
   end
   u = xdot.*cos(h) + ydot.*sin(h);
   u(end+1) = u(end); % Match the vector size
   
   % Find time intervals
   condition = u > minimum_u;
   [time_start, time_end] = find_time_intervals_to_remove(reference_struct, condition);

   % Remove the time intervals
   for i=1:length(time_start)
       time_interval = [time_start(i), time_end(i)];
       structs = remove_time_interval(time_interval, structs);
   end
end