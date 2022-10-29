function structs = remove_time_interval(time_interval, structs)
% For the given time interval, remove the corresponding section in each of
% the structs. Note that the structs do not have to be time synced. The
% time interval should have format [start_time, end_time]
    field_names = fieldnames(structs);   
   for i = 1:length(field_names)
        condition = or(structs.(field_names{i}).time < time_interval(1), (time_interval(2) < structs.(field_names{i}).time));
        field_names_2 = fieldnames(structs.(field_names{i}));
        for j = 2:length(field_names_2)
            structs.(field_names{i}).(field_names_2{j}) = structs.(field_names{i}).(field_names_2{j})(condition);
        end
        structs.(field_names{i}).time = structs.(field_names{i}).time(condition);
   end
end