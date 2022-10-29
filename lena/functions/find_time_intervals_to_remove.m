function [time_start, time_end] = find_time_intervals_to_remove(reference_struct, condition)
    % Find the time intervals for which the condition is not met. Note that
    % condition is a logical arrray that is the same length as the
    % reference_struct.time

   % Set time vector
   time = reference_struct.time;

   % Record the time interval end points for removal
   time_start = zeros(1, length(time));
   time_start_idx = 1;
   time_end = zeros(1, length(time));
   time_end_idx = 1;

   % Check the first value for a starting points
   if ~condition(1)
       time_start(1) = time(1);
       time_start_idx = time_start_idx + 1;
   end

   % Find the start and end points
   for i = 1:length(reference_struct.time)-1
       % Check for time interval end
        if ~condition(i) && condition(i+1)
            time_end(time_end_idx) = time(i);
            time_end_idx = time_end_idx + 1;
        end
        % Check for time interval start
        if condition(i) && ~condition(i+1)
            time_start(time_start_idx) = time(i+1);
            time_start_idx = time_start_idx + 1;
        end
   end

   % If not equal, we have one more start index than we do an end index
   if (time_start_idx ~= time_end_idx)
       time_end(time_end_idx) = time(end);
   end
end