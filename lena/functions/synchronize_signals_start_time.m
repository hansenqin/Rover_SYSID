function structs = synchronize_signals_start_time(structs)
   % Shifts the time of all of the structs to start at t = 0
   field_names = fieldnames(structs);
   for i = 1:length(field_names)
        structs.(field_names{i}).time = structs.(field_names{i}).time - structs.(field_names{i}).time(1);
    end
end