function [time_start, time_end] = divide_into_time_intervals(reference_struct, min_time_between_intervals, min_time_interval_length, expected_num_traj, dt)
    % Divides the data into trajectories based on time intervals. There is
    % a minimum length for a time interval, and a minimum time difference
    % between two intervals
    time = reference_struct.time;
    endpoints = zeros(1, expected_num_traj*2);
    endpoint_idx = 1;
    
    % First endpoint is the first data point
    endpoints(1) = time(endpoint_idx); 
    endpoint_idx = endpoint_idx + 1;

    % Find the other endpoints based on the interval requirements
    for i = 2:length(time)-1
        % Check the next time data point against the previous one to check
        % if they are far enough away to be endpoints regard of time
        % interval length
        if abs(time(i)-time(i-1)) > min_time_between_intervals
            % Then both of these points may be end points. 
            % Check to see if the time was already recorded as an endpoint
            if endpoints(endpoint_idx-1) ~= time(i-1)
                endpoints(endpoint_idx) = time(i-1);
                endpoint_idx = endpoint_idx + 1;
            end
            % Add the start point
            endpoints(endpoint_idx) = time(i);
            endpoint_idx = endpoint_idx + 1;
        end
    end

    % Last endpoint is the last data point
    endpoints(endpoint_idx) = time(end); 

    % Now time_interval_endpoints is a series of endpoints. We need to find
    % which endpoints have trajectories between them, and we do this by
    % counting the number of data points that are between each of the times
    min_data_points = min_time_interval_length/dt;
    time_start = [];
    time_end = [];

    for i = 1:length(endpoints)-1
        % Check how many data points are between endpoint(i) and
        % endpoint(i+1)
        traj_points = time(and(endpoints(i) <= time, time <= endpoints(i+1)));
        num_points = length(traj_points);
        % If the interval has data and the interval is long enough, add the
        % endpoints as the start and end points of a trajectory
        if num_points > min_data_points
            time_start = [time_start, endpoints(i)];
            time_end = [time_end, endpoints(i+1)];
        end
    end
end