function structs = remove_or_select_time_interval(time_interval, structs, usage_flag)
% For the given time interval, remove the corresponding section in each of
% the structs. Note that the structs do not have to be time synced. The
% time interval should have format [start_time, end_time]. The useage_flag
% == 1 for selecting the time interval, and ==0 for removing the time
% interval
    field_names = fieldnames(structs);   
   for i = 1:length(field_names)
       % Choose to remove or select the trajectory
        if usage_flag % selecting time interval
            condition = and(time_interval(1) <= structs.(field_names{i}).time, structs.(field_names{i}).time <= time_interval(2));
        else % removing time interval
        condition = or(structs.(field_names{i}).time < time_interval(1), (time_interval(2) < structs.(field_names{i}).time));
        end
        % Keep the struct where the condition is true
        field_names_2 = fieldnames(structs.(field_names{i}));
        for j = 2:length(field_names_2)
            structs.(field_names{i}).(field_names_2{j}) = structs.(field_names{i}).(field_names_2{j})(condition);
        end
        structs.(field_names{i}).time = structs.(field_names{i}).time(condition);
   end
end