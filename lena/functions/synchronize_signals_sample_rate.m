function structs = synchronize_signals_sample_rate(reference_struct, structs)
   % Interpolates the given structs to be at the same time
    % as the standard time.
    standard_time = reference_struct.time;
    field_names = fieldnames(structs);
    for i = 1:length(field_names)
        field_names_2 = fieldnames(structs.(field_names{i}));
        time = structs.(field_names{i}).time;
        for j = 2:length(field_names_2)
            if ~isempty(structs.(field_names{i}).(field_names_2{j}))
            structs.(field_names{i}).(field_names_2{j}) = interp1(time, structs.(field_names{i}).(field_names_2{j}), standard_time, 'linear', 'extrap'); 
            end
        end
        structs.(field_names{i}).time = standard_time;
    end
end