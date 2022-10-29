function structs = remove_via_condition(reference_struct, structs, condition)
   % Removes areas where the condition fails for the given structs. Note
   % that the times of the structs do not have to match.

   % Find time intervals
   [time_start, time_end] = find_time_intervals_to_remove(reference_struct, condition);

   % Remove the time intervals
   for i=1:length(time_start)
       time_interval = [time_start(i), time_end(i)];
       structs = remove_time_interval(time_interval, structs);
   end
end