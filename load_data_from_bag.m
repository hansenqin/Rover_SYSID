function [gen_param_out_data, parameter_data, polygons_data_time_output, polygons_data, state_data, rd_data, ud_data, maneuver_type_data, auto_flag_data, mocap_data, sensor_data, motor_cmd, servo_cmd, imu_data, data_est_uvrw, slam_h] = load_data_from_bag(bag_file)
    %% some state info
    bSel = select(bag_file,'Topic','/state_out/rover_debug_state_out');
    time_m = table2array(bSel.MessageList(:,1))*1e9;
    msgStructs = readMessages(bSel,'DataFormat','struct');
    time = cell2mat(cellfun(@(s)cast(s.Header.Stamp.Sec,'double')*1e9+cast(s.Header.Stamp.Nsec,'double'),msgStructs,'uni',0));
    
    rd_data = cell2mat(cellfun(@(s)s.Rd,msgStructs,'uni',0));
    ud_data = cell2mat(cellfun(@(s)s.Ud,msgStructs,'uni',0));
    state_data_X = cell2mat(cellfun(@(s)s.X,msgStructs,'uni',0));
    state_data_Y = cell2mat(cellfun(@(s)s.Y,msgStructs,'uni',0));
    state_data_H = cell2mat(cellfun(@(s)s.H,msgStructs,'uni',0));
    state_data_U = cell2mat(cellfun(@(s)s.U,msgStructs,'uni',0));
    state_data_V = cell2mat(cellfun(@(s)s.V,msgStructs,'uni',0));
    state_data_R = cell2mat(cellfun(@(s)s.R,msgStructs,'uni',0));
    state_data_Ud = cell2mat(cellfun(@(s)s.Ud,msgStructs,'uni',0));
    state_data_Vd = cell2mat(cellfun(@(s)s.Vd,msgStructs,'uni',0));
    state_data_Rd = cell2mat(cellfun(@(s)s.Rd,msgStructs,'uni',0));
    

    auto_flag_data = cell2mat(cellfun(@(s)s.RunningAuto,msgStructs,'uni',0));

    
    u0_param = double(cell2mat(cellfun(@(s)s.U0,msgStructs,'uni',0)));
    Au_param = double(cell2mat(cellfun(@(s)s.Au,msgStructs,'uni',0)));
    Ay_param = double(cell2mat(cellfun(@(s)s.Ay,msgStructs,'uni',0)));
    type_param = double(cell2mat(cellfun(@(s)s.ManuType,msgStructs,'uni',0)));


    if isfield(msgStructs{1},'ManuType')
        maneuver_type_data = double(cell2mat(cellfun(@(s)s.ManuType,msgStructs,'uni',0)));
    else
        maneuver_type_data = zeros(size(r_data));
    end


    state_data = [time, state_data_X, state_data_Y, state_data_H, ...
        state_data_U, state_data_V, state_data_R, state_data_Ud, ...
        state_data_Vd, state_data_Rd];

    auto_flag_data = [time, auto_flag_data];

    parameter_data = [time, u0_param, Au_param, Ay_param, type_param];
    %% SLAM heading
    bSel = select(bag_file,'Topic','/tf');
    time_m = table2array(bSel.MessageList(:,1))*1e9;
    
    if size(time_m) > 0
        msgStructs = readMessages(bSel,'DataFormat','struct');
        time = cell2mat(cellfun(@load_tf_transform_t,msgStructs,'uni',0));
        
        data = cell2mat(cellfun(@load_tf_transform_h, msgStructs,'uni',0));

        slam_h = [time data];
    else
        slam_h = 0;
    end
%     slam_h = 0;
    
    %% Sensors motor spd
    bSel = select(bag_file,'Topic','/vesc/sensors/core');
    time_m = table2array(bSel.MessageList(:,1))*1e9;
    if size(time_m) > 0
        msgStructs = readMessages(bSel,'DataFormat','struct');
        time = cell2mat(cellfun(@(s)cast(s.Header.Stamp.Sec,'double')*1e9+cast(s.Header.Stamp.Nsec,'double'),msgStructs,'uni',0));
        
        data = cell2mat(cellfun(@(s)s.State.Speed,msgStructs,'uni',0));

        sensor_data = [time data];
    else
        sensor_data = [time zeros(size(auto_flag_data))];
    end

    %% Mocap
    bSel = select(bag_file,'Topic','/mocap');
    time_m = table2array(bSel.MessageList(:,1))*1e9;
    if size(time_m) > 0
        msgStructs = readMessages(bSel,'DataFormat','struct');
        time = cell2mat(cellfun(@(s)cast(s.Header.Stamp.Sec,'double')*1e9+cast(s.Header.Stamp.Nsec,'double'),msgStructs,'uni',0));
        

        mocap_data = [];
        data = cell2mat(cellfun(@(s)s.Pose.Position.X,msgStructs,'uni',0));
        mocap_data = [mocap_data data];
        data = cell2mat(cellfun(@(s)s.Pose.Position.Y,msgStructs,'uni',0));
        mocap_data = [mocap_data data];
        data = cell2mat(cellfun(@(s)s.Pose.Position.Z,msgStructs,'uni',0));
        mocap_data = [mocap_data data];
        data = cell2mat(cellfun(@(s)s.Pose.Orientation.X,msgStructs,'uni',0));
        mocap_data = [mocap_data data];
        data = cell2mat(cellfun(@(s)s.Pose.Orientation.Y,msgStructs,'uni',0));
        mocap_data = [mocap_data data];
        data = cell2mat(cellfun(@(s)s.Pose.Orientation.Z,msgStructs,'uni',0));
        mocap_data = [mocap_data data];
        data = cell2mat(cellfun(@(s)s.Pose.Orientation.W,msgStructs,'uni',0));
        mocap_data = [mocap_data data];

        mocap_data = [time mocap_data];
        mocap_data = [mocap_data(:,1) zeros(size(mocap_data,1), 3) mocap_data(:,2:end)];

    else

    mocap_data = [time zeros(size(time,1), 6) zeros(size(time,1), 3) ones(size(time,1),1)];
    end

    %% motor_cmd
    bSel = select(bag_file,'Topic','/vesc/commands/motor/speed');
    time = table2array(bSel.MessageList(:,1))*1e9;
    msgStructs = readMessages(bSel,'DataFormat','struct');
    data = cell2mat(cellfun(@(s)s.Data,msgStructs,'uni',0));

    motor_cmd = [time data];

    %% delta
    bSel = select(bag_file,'Topic','/vesc/commands/servo/position');
    time = table2array(bSel.MessageList(:,1))*1e9;
    msgStructs = readMessages(bSel,'DataFormat','struct');
    data = cell2mat(cellfun(@(s)s.Data,msgStructs,'uni',0));

    servo_cmd = [time data];
    %% imu
    bSel = select(bag_file,'Topic','/imu/data');
    time_m = table2array(bSel.MessageList(:,1))*1e9;
    if size(time_m) > 0
    msgStructs = readMessages(bSel,'DataFormat','struct');
    time = cell2mat(cellfun(@(s)cast(s.Header.Stamp.Sec,'double')*1e9+cast(s.Header.Stamp.Nsec,'double'),msgStructs,'uni',0));
    

    data_uacc = cell2mat(cellfun(@(s)s.LinearAcceleration.X,msgStructs,'uni',0));
    data_vacc = cell2mat(cellfun(@(s)s.LinearAcceleration.Y,msgStructs,'uni',0));
    data_yaw  = cell2mat(cellfun(@(s)s.AngularVelocity.Z,msgStructs,'uni',0));
    imu_data = [time data_uacc data_vacc data_yaw];
    else
        imu_data = [time zeros(size(time,1),3)];
    end
    %% state estimator if avaliable
    bSel = select(bag_file,'Topic','/state_estimator/states');
    if size(bSel.MessageList,1) >1 
        time_m = table2array(bSel.MessageList(:,1))*1e9;
        msgStructs = readMessages(bSel,'DataFormat','struct');
        time = cell2mat(cellfun(@(s)cast(s.Header.Stamp.Sec,'double')*1e9+cast(s.Header.Stamp.Nsec,'double'),msgStructs,'uni',0));
        
        if isfield(msgStructs{1},'U')
            data_est_u = cell2mat(cellfun(@(s)s.U,msgStructs,'uni',0));
            data_est_v = cell2mat(cellfun(@(s)s.V,msgStructs,'uni',0));
            data_est_r  = cell2mat(cellfun(@(s)s.R,msgStructs,'uni',0));
            data_est_w  = cell2mat(cellfun(@(s)s.W,msgStructs,'uni',0));
            data_est_uvrw = [time data_est_u data_est_v data_est_r data_est_w];
        else 
            data_est_u = cell2mat(cellfun(@(s)s.Point.X,msgStructs,'uni',0));
            data_est_v = cell2mat(cellfun(@(s)s.Point.Y,msgStructs,'uni',0));
            data_est_r  = cell2mat(cellfun(@(s)s.Point.Z,msgStructs,'uni',0));
            data_est_uvrw = [time data_est_u data_est_v data_est_r zeros(size(time,1), 1)];
        end
    else
        data_est_uvrw = [time zeros(size(time,1), 4)];
    end

    bSel = select(bag_file,'Topic','/zonotope_visualization');
    if size(bSel.MessageList,1) >1 
        time_m = table2array(bSel.MessageList(:,1))*1e9;
        msgStructs = readMessages(bSel,'DataFormat','struct');
        time = cell2mat(cellfun(@(s)cast(s.Header.Stamp.Sec,'double')*1e9+cast(s.Header.Stamp.Nsec,'double'),msgStructs,'uni',0));
        
        polygons_data_in = cellfun(@(s)s.Polygons,msgStructs,'uni',0);
        polygons_data = {};
        polygons_data_time_output = time;
        for time_idx = 1:length(polygons_data_in)
            polygons_data_time_idx = polygons_data_in{time_idx};
            polygons_data{time_idx} = zeros(2, 6*length(polygons_data_time_idx));
            for i = 1:length(polygons_data_time_idx)
                x = cell2mat({polygons_data_time_idx(i).Polygon.Points(:).X});
                y = cell2mat({polygons_data_time_idx(i).Polygon.Points(:).Y});
                polygons_data{time_idx}(:,i*6-5:i*6) = double([x x(1) nan;y y(1) nan]);
            end
        end
    else
        polygons_data_time_output = [];
        polygons_data =[];
    end
    
    mocap_data(:,1)     =    (mocap_data(:,1)) * 1e-9 - start_time_offset;
    state_data(:,1)     =    (state_data(:,1)) * 1e-9 - start_time_offset;
    slam_h(:,1)         =    (slam_h(:,1)) * 1e-9 - start_time_offset;
    servo_cmd(:,1)      =    (servo_cmd(:,1)) * 1e-9- start_time_offset;
    auto_flag_data(:,1) =    (auto_flag_data(:,1)) * 1e-9-  start_time_offset;
    motor_cmd(:,1)      =    (motor_cmd(:,1)) * 1e-9- start_time_offset;

    if ~isempty(motor_cmd)
    motor_cmd(:,2) =  motor_cmd(:,2)/2072;
    sensor_data(:,1) =       (sensor_data(:,1)) * 1e-9 - start_time_offset;
    sensor_data(:,2) = sensor_data(:,2)/2072;
    imu_data(:,1)    =       (imu_data   (:,1)) * 1e-9 - start_time_offset;
    data_est_uvrw(:,1) = (data_est_uvrw(:,1)) * 1e-9 - start_time_offset;
    end


end