function structs = remove_negative_encoder_velocity(structs, minimum_u)
   % Removes areas of negative/very small u for the given structs. Note
   % that the times of the structs do not have to match. The reference
   % struct must be the vehicle encoder

   % Find where the reference struct has negative/small u
   condition = structs.wheel_encoder.encoder_velocity > minimum_u;

   % Find time intervals
   [time_start, time_end] = find_time_intervals_to_remove(structs.wheel_encoder, condition);

   % Remove the time intervals
   for i=1:length(time_start)
       time_interval = [time_start(i), time_end(i)];
       structs = remove_time_interval(time_interval, structs);
   end
end