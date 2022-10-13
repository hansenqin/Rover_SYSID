classdef tire_curve_sysID_helper_class < handle
    %Authors: Hansen Qin, Lena Trang, Vishrut Kaushik
    %Email: qinh@umich.edu
    %Created: 5/17/2022
    %Modified: 6/2/2022
    %
    %Purpose: solve for the lateral tire characetristic curve coefficients
    %         given a ros obj.bag with vehicle states and vehicle commands.
    %         The tire characterisitc curve is in the form:
    %         F_lateral = C1*tanh(C2*slip_angle)
    
    
   
   properties
        pass_band
        lf
        lr
        m
        Izz
        servo_offset
        rw
        g
        robot_frame
        
        %rosobj.bag
        bag
        
        % structs that store signals 
        vehicle_states = struct('time', 0, 'x', 0, 'y', 0, 'h', 0, 'u', 0, 'v', 0, 'r', 0, 'w', 0, 'delta_cmd', 0);
%         vehicle_delta_command = struct('time', 0, 'delta_cmd', 0);
        vehicle_encoder = struct('time', 0, 'encoder_position', 0, 'encoder_velocity', 0, 'edited_w', 0);
        vehicle_motor_current_command = struct('time', 0, 'motor_current', 0);
        desired_states = struct('time', 0, 'ud', 0, 'vd', 0, 'rd', 0);
        auto_flag_data = struct('time', 0, 'auto_flag', 0);
        vehicle_pose_from_slam = struct('time', 0, 'x', 0, 'y', 0, 'h', 0);
%         vehicle_pose_from_mocap = struct('time', 0, 'x', 0, 'y', 0, 'h', 0);
        vehicle_velocities_derived_from_slam = struct('time', 0, 'u', 0, 'v', 0, 'r', 0);
        vehicle_acceleration_derived_from_slam = struct('time', 0, 'udot', 0, 'vdot', 0, 'rdot', 0);
        vehicle_angular_velocity = struct('time', 0, 'w', 0);
        vehicle_states_from_slam = struct('time', 0, 'x', 0, 'y', 0, 'h', 0, 'u', 0, 'v', 0, 'r', 0, 'udot', 0, 'vdot', 0, 'rdot', 0);
        rover_debug_states = struct('time', 0, 'w', 0, 'delta_cmd', 0); 
        trajectories = struct('trajectory', 0);
        trajectory = struct('time', 0, 'x', 0, 'y', 0, 'h', 0, 'u', 0, 'v', 0, 'udot', 0, 'vdot', 0, 'encoder_velocity', 0);
        all_trajectory = struct('time', 0, 'x', 0, 'y', 0, 'h', 0, 'u', 0, 'v', 0, 'r', 0, 'udot', 0, 'vdot', 0, 'rdot', 0, 'encoder_velocity', 0);
        
        % signal stuff
        start_time_offset;
        standard_time;  %standard usd to synchronize all signals
        
        
        % auto flags on
        auto_flag_on
        
        
   end
   
   methods
           function obj = tire_curve_sysID_helper_class(bag_name, rover_config, varargin)
                close all;
                % read bag
                obj.bag = rosbag(bag_name);
                
                rover_config = read_json(obj, rover_config);
                
                % set vehicle constants
                obj.pass_band = 50;
                obj.lf = rover_config.lf;
                obj.lr = rover_config.lr;
                obj.m = rover_config.m;
                obj.Izz = rover_config.Izz;
                obj.servo_offset = rover_config.servo_offset;
                obj.rw = rover_config.rw;
                obj.g = rover_config.g;
                obj.robot_frame = string(rover_config.robot_frame);
                % set default flags
                obj.auto_flag_on = 0;
    %             obj.removeNegativeUFilterValue = 0.4; %Filters out u values under 0.4 m/s
               
               
                % parse optional intputs
                for i = 1:2:length(varargin) % work for a list of name-value pairs
                    if ischar(varargin{i}) || isstring(varargin{i}) % check if is character
                        obj.(varargin{i}) = varargin{i+1}; % override or add parameters to structure.
                    end
                end
                
                %Load data
                load_vehicle_states_data(obj);
                load_rover_debug_states(obj);
%                 load_vehicle_pose_from_slam_data(obj);
                load_zonotope_data(obj);
                load_commands(obj);
                load_imu_data(obj); 
                load_param_gen_data(obj);
                load_auto_flag(obj);
                load_mocap_data(obj);
                synchronize_encoder_with_trajectories(obj);
                compute_velocity_and_acceleration_from_slam_data(obj, false, true); % Arg2 = Enable/Disable Tracking Plot || Arg2 = Enable/Disable Filter +ve u values
    
                
                % Sync signals
                % rate, and all other topics will have their sample rates
                % synchronized to the standard sampling rate
    %             structs_to_sync = ["vehicle_encoder", "rover_debug_states"];
    %             synchronize_signals_sample_rate(obj, structs_to_sync);
                
                
    %             obj.plot_tracking_data();
                
                % Low pass signals
    %             structs_to_low_pass = ["vehicle_states"];
    %             lowpass_signals(obj, structs_to_low_pass)
                
                % Auto filter signals 
    %             if obj.auto_flag_on
    % %                 structs_to_auto_filter = ["vehicle_states", "vehicle_delta_command", "vehicle_motor_current_command", "desired_states"];
    %                 structs_to_auto_filter = ["vehicle_states", "vehicle_motor_current_command", "vehicle_encoder", "desired_states"];
    %                 auto_filter(obj, structs_to_auto_filter)
    %             end
                
           end
           
           
           function json = read_json(obj, fname)
                fid = fopen(fname);
                raw = fread(fid,inf); 
                str = char(raw'); 
                fclose(fid); 
                json = jsondecode(str);
           end
           
           
    %        function obj = set_standard_time(obj, time)
    %            obj.standard_time = time;
    %        end
           
           
           function obj = synchronize_signals_sample_rate(obj, structs)
    
                for i = 1:length(structs)
                    field_names = fieldnames(obj.(structs(i)));
                    time = obj.(structs(i)).time;
                    for j = 2:length(field_names)
                        obj.(structs(i)).(field_names{j}) = interp1(time(:),obj.(structs(i)).(field_names{j}),obj.standard_time(:),'linear', 'extrap'); % 'extrap'
                    end
                    obj.(structs(i)).time = obj.standard_time(:);
                end
           end
           
           function obj = synchronize_encoder_with_trajectories(obj)
                
% %                 for traj_idx = 1:length(obj.trajectories)
% %                     standard_time = obj.trajectories(traj_idx).time;
% %                     time = obj.vehicle_encoder.time';
% %                     time = time - time(1) + standard_time(1);
% %                     obj.trajectories(traj_idx).encoder_velocity = interp1(time(:), obj.vehicle_encoder.encoder_velocity, standard_time(:), 'linear', 'extrap')'; % 'extrap'            end
% %                 end


                    standard_time = obj.vehicle_states_from_slam.time;
                    time = obj.vehicle_encoder.time';
                    time = time - time(1) + standard_time(1);
                    obj.vehicle_encoder.edited_w = interp1(time(:), obj.vehicle_encoder.encoder_velocity, standard_time(:), 'linear', 'extrap')'; % 'extrap'            end
               

                
           end
           
           function synchronize_signals_time(obj)
               %Might not be needed
               
           end
           
                  
           function auto_filter(obj, structs)
               % Only keeps the data from the automatic driving mode instead
               % of manual driving
               for i = 1:length(structs) 
                    field_names = fieldnames(obj.(structs(i)));
                    for j = 1:length(field_names)
                        signal = obj.(structs(i)).(field_names{j});
                        obj.(structs(i)).(field_names{j}) = signal(logical(obj.auto_flag_data.auto_flag));
                    end
               end
           end
           
           
           function lowpass_signals(obj, structs)
                 % Only keeps the data from the automatic trajectory running
               for i = 1:length(structs) 
                    field_names = fieldnames(obj.(structs(i)));
                    for j = 1:length(field_names)
                        obj.(structs(i)).(field_names{j}) = lowpass(obj.(structs(i)).(field_names{j}), 0.1, obj.pass_band);
                        
                    end
               end
           end
           
           function obj = load_rover_debug_states(obj)
                bSel = select(obj.bag,'Topic','/state_out/rover_debug_state_out');
                msgStructs = readMessages(bSel,'DataFormat','struct');
                obj.rover_debug_states.time = cell2mat(cellfun(@(s)cast(s.Header.Stamp.Sec,'double')*1e9+cast(s.Header.Stamp.Nsec,'double'),msgStructs,'uni',0))*1e-9;
                obj.rover_debug_states.w = cell2mat(cellfun(@(s)s.W,msgStructs,'uni',0));
                obj.rover_debug_states.delta_cmd = cell2mat(cellfun(@(s)s.DeltaCmd,msgStructs,'uni',0));
           end
    
           function obj = load_vehicle_states_data(obj)
                bSel = select(obj.bag,'Topic','/state_out/rover_debug_state_out');
                msgStructs = readMessages(bSel,'DataFormat','struct');
                obj.vehicle_states.time = cell2mat(cellfun(@(s)cast(s.Header.Stamp.Sec,'double')*1e9+cast(s.Header.Stamp.Nsec,'double'),msgStructs,'uni',0))*1e-9;
                obj.vehicle_states.x = cell2mat(cellfun(@(s)s.X,msgStructs,'uni',0));
                obj.vehicle_states.y = cell2mat(cellfun(@(s)s.Y,msgStructs,'uni',0));
                obj.vehicle_states.h = cell2mat(cellfun(@(s)s.H,msgStructs,'uni',0));
                obj.vehicle_states.u = cell2mat(cellfun(@(s)s.U,msgStructs,'uni',0));
                obj.vehicle_states.v = cell2mat(cellfun(@(s)s.V,msgStructs,'uni',0));
                obj.vehicle_states.r = cell2mat(cellfun(@(s)s.R,msgStructs,'uni',0));
                obj.vehicle_states.w = cell2mat(cellfun(@(s)s.W,msgStructs,'uni',0));
    
                obj.vehicle_states.delta_cmd = cell2mat(cellfun(@(s)s.DeltaCmd,msgStructs,'uni',0));
    
                
                obj.desired_states.time = cell2mat(cellfun(@(s)cast(s.Header.Stamp.Sec,'double')*1e9+cast(s.Header.Stamp.Nsec,'double'),msgStructs,'uni',0))*1e-9;
                obj.desired_states.ud = cell2mat(cellfun(@(s)s.Ud,msgStructs,'uni',0));
                obj.desired_states.vd = cell2mat(cellfun(@(s)s.Vd,msgStructs,'uni',0));
                obj.desired_states.rd = cell2mat(cellfun(@(s)s.Rd,msgStructs,'uni',0));
    
                bSel = select(obj.bag,'Topic','/joint_states');
                msgStructs = readMessages(bSel,'DataFormat','struct');
                obj.vehicle_encoder.time = cell2mat(cellfun(@(s)cast(s.Header.Stamp.Sec,'double')*1e9+cast(s.Header.Stamp.Nsec,'double'),msgStructs,'uni',0))*1e-9;
                obj.vehicle_encoder.encoder_position = cell2mat(cellfun(@(s)s.Position,msgStructs,'uni',0));
                
                obj.vehicle_encoder.encoder_velocity = (diff(obj.vehicle_encoder.encoder_position)./diff(obj.vehicle_encoder.time))';
                obj.vehicle_encoder.encoder_velocity  = -obj.vehicle_encoder.encoder_velocity*2*pi/(4096);
                obj.vehicle_encoder.encoder_velocity = smoothdata(obj.vehicle_encoder.encoder_velocity, 'gaussian', 20);
                
                obj.vehicle_encoder.encoder_position = obj.vehicle_encoder.encoder_position(1:end-1);
                obj.vehicle_encoder.time = obj.vehicle_encoder.time(1:end-1);
           end
           
           function obj = load_vehicle_pose_from_slam_data(obj)
                bSel = select(obj.bag, 'Topic', '/tf');
                tf = cell2mat(readMessages(bSel, 'DataFormat', 'struct'));
                obj.vehicle_pose_from_slam.time = [];
                obj.vehicle_pose_from_slam.x = [];
                obj.vehicle_pose_from_slam.y = [];
                obj.vehicle_pose_from_slam.h = [];
                for i = 1:size(tf)
                    if size(tf(i).Transforms, 2) > 0
                        if tf(i).Transforms.Header.FrameId == "map" && tf(i).Transforms.ChildFrameId == obj.robot_frame
                            time = (cast(tf(i, 1).Transforms.Header.Stamp.Sec, 'double')*1e9 + cast(tf(i, 1).Transforms.Header.Stamp.Nsec, 'double'))*1e-9;
                            obj.vehicle_pose_from_slam.time = [obj.vehicle_pose_from_slam.time, time];
                            obj.vehicle_pose_from_slam.x = [obj.vehicle_pose_from_slam.x, tf(i).Transforms.Transform.Translation.X];
                            obj.vehicle_pose_from_slam.y = [obj.vehicle_pose_from_slam.y, tf(i).Transforms.Transform.Translation.Y];
                            q = [tf(i).Transforms.Transform.Rotation.W, tf(i).Transforms.Transform.Rotation.X, tf(i).Transforms.Transform.Rotation.Y, tf(i).Transforms.Transform.Rotation.Z];
                            [z, y, x] = quat2angle(q);
                            obj.vehicle_pose_from_slam.h = [obj.vehicle_pose_from_slam.h, z];
                        end
                    end
                end
           end

           function obj = load_mocap_data(obj)

                obj.vehicle_pose_from_slam.time = [];
                obj.vehicle_pose_from_slam.x = [];
                obj.vehicle_pose_from_slam.y = [];
                obj.vehicle_pose_from_slam.h = [];
                
                time = [];
                x = [];
                y = [];
                h = [];

                bSel = select(obj.bag, 'Topic', '/mocap');
                mocap = cell2mat(readMessages(bSel, 'DataFormat', 'struct'));

                for i = 1:size(mocap)
                    x = [x, mocap(i).Pose.Position.X*1e-3];
                    y = [y, -mocap(i).Pose.Position.Z*1e-3];
                    t = (cast(mocap(i, 1).Header.Stamp.Sec, 'double')*1e9 + cast(mocap(i, 1).Header.Stamp.Nsec, 'double'))*1e-9;
                    time = [time, t];

                    q = [mocap(i).Pose.Orientation.W, mocap(i).Pose.Orientation.X, mocap(i).Pose.Orientation.Y, mocap(i).Pose.Orientation.Z];
                    [z, y_crap, x_crap] = quat2angle(q);
                    h = [h, z];           
                end

                obj.vehicle_pose_from_slam.time = time;
                obj.vehicle_pose_from_slam.x = x;
                obj.vehicle_pose_from_slam.y = y;
                obj.vehicle_pose_from_slam.h = h;

           end
    
           function obj = compute_temporary_velocity_from_slam_data(obj)
               x = obj.vehicle_pose_from_slam.x;
               y = obj.vehicle_pose_from_slam.y;
               h = obj.vehicle_pose_from_slam.h;
               time = obj.vehicle_pose_from_slam.time;
    
               u = [];
               v = [];
               r = [];
               for i = 1:length(x)-1
                    u = [u, (x(i+1) - x(i)) / (time(i+1) - time(i) + 0.001)];
                    v = [v, (y(i+1) - y(i)) / (time(i+1) - time(i) + 0.001)];
                    r = [r, (h(i+1) - h(i)) / (time(i+1) - time(i) + 0.001)];
               end
    
               obj.vehicle_velocities_derived_from_slam.u = u;
               obj.vehicle_velocities_derived_from_slam.v = v;
               obj.vehicle_velocities_derived_from_slam.r = r;
               obj.vehicle_velocities_derived_from_slam.time = time(1:end-1);
           end
    
           function obj = compute_temporary_acceleration_from_slam_data(obj)
               u = obj.vehicle_velocities_derived_from_slam.u;
               v = obj.vehicle_velocities_derived_from_slam.v;
               r = obj.vehicle_velocities_derived_from_slam.r;
               time = obj.vehicle_velocities_derived_from_slam.time;
    
               udot = [];
               vdot = [];
               rdot = [];
               for i = 1:length(u)-1
                    udot = [udot, (u(i+1) - u(i)) / (time(i+1) - time(i) + 0.001)];
                    vdot = [vdot, (v(i+1) - v(i)) / (time(i+1) - time(i) + 0.001)];
                    rdot = [rdot, (r(i+1) - r(i)) / (time(i+1) - time(i) + 0.001)];
               end
    
               obj.vehicle_acceleration_derived_from_slam.udot = udot;
               obj.vehicle_acceleration_derived_from_slam.vdot = vdot;
               obj.vehicle_acceleration_derived_from_slam.rdot = rdot;
               obj.vehicle_acceleration_derived_from_slam.time = time(1:end-2);
           end
    
           function obj = get_trajectories_from_slam(obj)
                obj.trajectories = [];
                start_idx = 1;
                end_idx = 1;
                threshHoldDistance = 1.0;
                numOfTrajectories = 0;
                while start_idx <= length(obj.vehicle_states_from_slam.x)-1
                    end_idx = start_idx + 1;
                    for i = start_idx+1:length(obj.vehicle_states_from_slam.x)-1
                        distance = ((obj.vehicle_states_from_slam.x(i) - obj.vehicle_states_from_slam.x(i+1))^2 + (obj.vehicle_states_from_slam.y(i) - obj.vehicle_states_from_slam.y(i+1))^2)^0.5;  
                        if distance > threshHoldDistance
                            end_idx = i;
                            break;
                        end
                    end
                    if (end_idx - start_idx) > 10
                        obj.trajectory.x = obj.vehicle_states_from_slam.x(start_idx:end_idx);
                        obj.trajectory.y = obj.vehicle_states_from_slam.y(start_idx:end_idx);
                        obj.trajectory.h = obj.vehicle_states_from_slam.h(start_idx:end_idx);
                        obj.trajectory.time = obj.vehicle_states_from_slam.time(start_idx:end_idx);
                        obj.trajectories = [obj.trajectories, obj.trajectory];
                        numOfTrajectories = numOfTrajectories + 1;
                    end
                    start_idx = end_idx + 1;
                end
    
                disp([numOfTrajectories, " trajectories were detected"]);
           end
    
           function obj = smoothen_trajectories(obj)
                
           end
    
    %        function obj = fit_bezier_curve(obj)
    %        
    %        end
    
           function obj = compute_velocity_and_acceleration_from_slam_data(obj, displayPlot, removeNegativeU)
               obj.compute_temporary_velocity_from_slam_data();
               obj.compute_temporary_acceleration_from_slam_data();
               
    %            To sync time
               obj.vehicle_states_from_slam.time = obj.vehicle_pose_from_slam.time(1:end-2);
               obj.vehicle_states_from_slam.x = obj.vehicle_pose_from_slam.x(1:end-2);
               obj.vehicle_states_from_slam.y = obj.vehicle_pose_from_slam.y(1:end-2);
               obj.vehicle_states_from_slam.h = obj.vehicle_pose_from_slam.h(1:end-2);
    
               obj.vehicle_states_from_slam.u = obj.vehicle_velocities_derived_from_slam.u(1:end-1);
               obj.vehicle_states_from_slam.v = obj.vehicle_velocities_derived_from_slam.v(1:end-1);
               obj.vehicle_states_from_slam.r = obj.vehicle_velocities_derived_from_slam.r(1:end-1);
    
               obj.vehicle_states_from_slam.udot = obj.vehicle_acceleration_derived_from_slam.udot;
               obj.vehicle_states_from_slam.vdot = obj.vehicle_acceleration_derived_from_slam.vdot;
               obj.vehicle_states_from_slam.rdot = obj.vehicle_acceleration_derived_from_slam.rdot;
               
    %            disp(["Size before filter = ", size(obj.vehicle_states_from_slam.time) ]);
               removeNegativeUFilterValue = 0.0;
               if removeNegativeU
                    filter = obj.vehicle_states_from_slam.u > removeNegativeUFilterValue ;
                    field_names = fieldnames(obj.vehicle_states_from_slam);
                    for j = 1:length(field_names)
                        obj.vehicle_states_from_slam.(field_names{j}) = obj.vehicle_states_from_slam.(field_names{j})(filter);                
                    end      
               end
               
               get_trajectories_from_slam(obj);
               
    %            disp(["Size before filter = ", size(obj.vehicle_states_from_slam.time) ]);
               
    
               if displayPlot == true
                  obj.plot_tracking_data();
               end
           end
    
           function plot_tracking_data(obj)
                figure(1);
                hold on;
                scatter(obj.vehicle_states_from_slam.time, obj.vehicle_states_from_slam.x, 'r', 'DisplayName', 'SLAM X Smoothened', 'LineWidth', 2);
                scatter(obj.vehicle_states_from_slam.time, obj.vehicle_states_from_slam.u, 'b', 'DisplayName', 'SLAM U Smoothened', 'LineWidth', 2);
%                 plot(obj.vehicle_states_from_slam.udot, 'k', 'DisplayName', 'SLAM UDot Smoothened', 'LineWidth', 2);
                hold off;
                legend;
    %             figure(2);
    %             hold on;
    %             plot(obj.vehicle_states_from_slam.y, 'r', 'DisplayName', 'SLAM Y Smoothened', 'LineWidth', 2);
    %             plot(obj.vehicle_states_from_slam.v, 'b', 'DisplayName', 'SLAM V Smoothened', 'LineWidth', 2);
    %             plot(obj.vehicle_states_from_slam.vdot, 'k', 'DisplayName', 'SLAM VDot Smoothened', 'LineWidth', 2);
    %             hold off;
    %             legend;
    %             figure(3);
    %             hold on;
    %             plot(obj.vehicle_states_from_slam.h, 'r', 'DisplayName', 'SLAM H Smoothened', 'LineWidth', 2);
    %             plot(obj.vehicle_states_from_slam.r, 'b', 'DisplayName', 'SLAM R Smoothened', 'LineWidth', 2);
    %             plot(obj.vehicle_states_from_slam.rdot, 'k', 'DisplayName', 'SLAM RDot Smoothened', 'LineWidth', 2);
    %             hold off;
    
                legend;
           end
           
           function load_auto_flag(obj)
                bSel = select(obj.bag,'Topic','/state_out/rover_debug_state_out');
                msgStructs = readMessages(bSel,'DataFormat','struct');
                
                obj.auto_flag_data.auto_flag = double(cell2mat(cellfun(@(s)s.RunningAuto,msgStructs,'uni',0)));
                obj.auto_flag_data.time = cell2mat(cellfun(@(s)cast(s.Header.Stamp.Sec,'double')*1e9+cast(s.Header.Stamp.Nsec,'double'),msgStructs,'uni',0))*1e-9;
                obj.start_time_offset = obj.auto_flag_data.auto_flag(1,1);
                
           end
           
           
           function load_commands(obj)
                bSel_servo = select(obj.bag,'Topic','/vesc/sensors/servo_position_command');
                bSel_motor_current = select(obj.bag,'Topic','/vesc/sensors/core');
                msgStructs_servo = readMessages(bSel_servo,'DataFormat','struct');
                msgStructs_motor_current = readMessages(bSel_motor_current,'DataFormat','struct');
                
           
                % A -1 is multiplied witbh delta_cmd due to the set up of the
                % mechanical steering mechanism that inverts the direction of
                % the servo with the direction of the wheels.
    % % % % %             obj.vehicle_delta_command.delta_cmd = -cell2mat(cellfun(@(s)s.Data,msgStructs_servo,'uni',0));
    % % % % %             obj.vehicle_delta_command.time = table2array(bSel_servo.MessageList(:,1));
                
                obj.vehicle_motor_current_command.motor_current = cell2mat(cellfun(@(s)s.State.CurrentMotor,msgStructs_motor_current,'uni',0));
                obj.vehicle_motor_current_command.time = table2array(bSel_motor_current.MessageList(:,1));
                
           end
           
           
           function load_param_gen_data(obj)
               %TODO
           end
           
           
           function load_zonotope_data(obj)
               %TODO
           end
           
           
           function load_imu_data(obj)
               %TODO
           end
           
    
           function [xLinearCoef, fLinearCoef, rLinearCoef, lambda, alphaf, alphar, F_x, F_yf, F_yr] = get_tire_curve_coefficients(obj)
                time = obj.trajectory.time;
                x = obj.trajectory.x;
                y = obj.trajectory.y;
                h = obj.trajectory.h;
                u = obj.trajectory.u;
                v = obj.trajectory.v;
                r = obj.trajectory.r;
                udot = obj.trajectory.udot;
                vdot = obj.trajectory.vdot;
                rdot = obj.trajectory.rdot;
                
                 w = obj.trajectory.encoder_velocity;
    %            w = obj.rover_debug_states.w;
    %             delta=obj.vehicle_delta_command.delta_cmd(1:end-1)+obj.servo_offset;
                delta = obj.rover_debug_states.delta_cmd;
            
                % Calculate the longitudinal force
                % Current method
    %             F_x = 0.4*obj.vehicle_motor_current_command.motor_current - 6.0897*tanh(10.5921*obj.vehicle_states_from_slam.u);
    %             F_xr = Fx_est(1:end-1);
    %             F_xfw = Fx_est(1:end-1);
    
                % Calculate the slip ratio
                lambda = [];
                lambda_numerator = obj.rw.*w-u';
                for i = 1:length(udot)
                    if lambda_numerator(i) < 0
                        lambda = [lambda, lambda_numerator(i)./sqrt(u(i).^2+0.05)];
                    else
                        lambda = [lambda, lambda_numerator(i)./sqrt((obj.rw.*w(i)).^2+0.05)];
                    end
                end
                % Find slip angles
                alphaf = delta - (v + obj.lf.*r)./sqrt(u.^2+0.05);
                alphar = -(v-obj.lr.*r)./sqrt(u.^2+0.05);
                % Find longitudinal force: wheel encoder method
                F_x = obj.m.*(udot-v.*r);
                % Find latereal force
                F_yf = (obj.lr.*obj.m.*(vdot+u.*r)+rdot.*obj.Izz)./(obj.lf + obj.lr);
                F_yr=(obj.m.*(vdot+u.*r)-rdot*obj.Izz./obj.lf)./(1+obj.lr./obj.lf);
                % Sort the data
                [alphafSorted, If] = sort(alphaf);
                F_yfSorted = F_yf(If);
                [alpharSorted, Ir] = sort(alphar);
                F_yrSorted = F_yr(Ir);
                % Select and fit a curve to the data in the linear region 
                lambdaSelected = lambda(abs(lambda)<0.45);
                alphafSelected = alphafSorted(abs(alphafSorted)<0.2);
                alpharSelected = alpharSorted(abs(alpharSorted)<0.2);
                F_xSelected = F_x(abs(lambda)<0.45);
                F_yfSelected = F_yfSorted(abs(alphafSorted)<0.2);
                F_yrSelected = F_yrSorted(abs(alpharSorted)<0.2);
                
                % Longitudinal linear curve through (0,0) using fmincon
                x0 = 10;
                f = @(x) norm(obj.m.*obj.g.*lambdaSelected'.*x-F_xSelected);
                xLinearCoef = fmincon(f,x0);
                % Lateral linear curve through (0,0) using fmincon
                x0 = 1;
                f = @(x) norm(alphafSelected*x-F_yfSelected);
                fLinearCoef = fmincon(f,x0);
                f = @(x) norm(alpharSelected*x-F_yrSelected);
                rLinearCoef = fmincon(f,x0);
    %             Nonlinear curve using fmincon
    %             x0 = [1 1];
    %             f = @(x) norm(x(1)*tanh(x(2)*alphaf)-F_yf);
    %             fNonlinearCoef = fmincon(f,x0);
    %             r = @(x) norm(x(1)*tanh(x(2)*alphar)-F_yr);
    %             rNonlinearCoef = fmincon(r,x0);
           end
    
           function [xLinearCoef, lambda, F_x] = get_longitudnal_tire_curve_coefficients(obj)
                time = obj.trajectory.time;
                x = obj.trajectory.x;
                y = obj.trajectory.y;
                h = obj.trajectory.h;
                u = obj.trajectory.u;
                v = obj.trajectory.v;
                r = obj.trajectory.r;
                udot = obj.trajectory.udot;
                vdot = obj.trajectory.vdot;
                rdot = obj.trajectory.rdot;
                
                w = obj.trajectory.encoder_velocity;
                delta = obj.rover_debug_states.delta_cmd;
      
                % Calculate the slip ratio
                lambda = [];
                lambda_numerator = obj.rw.*w-u';
                for i = 1:length(udot)
                    if udot(i) < 0
                        lambda = [lambda, lambda_numerator(i)./sqrt(u(i).^2+0.05)];
                    else
                        lambda = [lambda, lambda_numerator(i)./sqrt((obj.rw.*w(i)).^2+0.05)];
                    end
                end
                % Find longitudinal force: wheel encoder method
                F_x = obj.m.*(udot-v.*r);
    
                % Select and fit a curve to the data in the linear region 
                lambdaSelected = lambda(abs(lambda)<0.45);
                F_xSelected = F_x(abs(lambda)<0.45);
                
                % Longitudinal linear curve through (0,0) using fmincon
                x0 = 10;
                f = @(x) norm(obj.m.*obj.g.*lambdaSelected'.*x - F_xSelected);
                xLinearCoef = fmincon(f,x0);
           end
    
           function plot_longitudnal_linear_tire_curve(obj, xLinearCoef, lambda, F_x)
                % Longitudinal
                figure(1);
                scatter(lambda, F_x);
                hold on;
                xLinear = @(t) obj.m*obj.g*xLinearCoef(1)*t/(obj.lf+obj.lr);
                t = -0.2:0.01:0.2;
                plot(t, xLinear(t), "LineWidth", 2);
                xlabel("Front Slip Ratio");
                ylabel("Front Longitudinal Tire Force (N)");
                title("Fxf+Fxr=mgl_r/l"+round(xLinearCoef(1),2)+"*lambda")
                xlim([-1,1]);
                hold off;
           end
    
    
           function plot_linear_tire_curve(obj, xLinearCoef, fLinearCoef, rLinearCoef, lambda, alphaf, alphar, F_x, F_yf, F_yr)
                % Longitudinal
                figure(1);
                scatter(lambda, F_x);
                hold on;
                xLinear = @(t) obj.m*obj.g*xLinearCoef(1)*t/(obj.lf+obj.lr);
                t = -0.2:0.01:0.2;
                plot(t, xLinear(t), "LineWidth", 2);
                xlabel("Front Slip Ratio");
                ylabel("Front Longitudinal Tire Force (N)");
                title("Fxf+Fxr=mgl_r/l"+round(fLinearCoef(1),2)+"*lambda")
                xlim([-1,1]);
                hold off;
    
                % Front tire lateral
    %             figure(2);
    %             scatter(alphaf, F_yf);
    %             hold on;
    %             fLinear = @(t) fLinearCoef(1)*t;
    %             t = -0.2:0.01:0.2;
    %             plot(t, fLinear(t), "LineWidth", 2);
    %             xlabel("Front Slip Angle");
    %             ylabel("Front Lateral Tire Force (N)");
    %             title("Fywf="+round(fLinearCoef(1),2)+"*alpha")
    %             xlim([-0.3,0.3]);
    %             hold off;
    %             
    %             % Rear tire lateral
    %             figure(3);
    %             scatter(alphar, F_yr);
    %             hold on;
    %             rLinear = @(t) rLinearCoef(1)*t;
    %             t = -0.2:0.01:0.2;
    %             plot(t, rLinear(t), "LineWidth", 2);
    %             xlabel("Rear Slip Angle");
    %             ylabel("Rear Lateral Tire Force (N)");
    %             title("Fywr="+round(rLinearCoef(1),2)+"*alpha");
    %             xlim([-0.3,0.3]);
                hold off;
           end
    
        function obj = fit_bezier_to_trajectory(obj)
            for i = 1:length(obj.trajectories)
               tracking = ["x", "y", "h"];
               % fit bezier for each x and y in the trajectory
               % note that a=x or a=y depending on what you are calculating
               % return y, a function that you can call as y(t), where t is the
               % time vector. P is also the coefficients of the bezier function
               for j = 1:length(tracking)
% tracking(i) == "h"
%                       obj.trajectories(i).(tracking(j))
%                    end
                   a = obj.trajectories(i).(tracking(j));
                   time = obj.trajectories(i).time;
                   time_original = time;
                   time_range = time(end)-time(1);
                   time = (time-time(1))./time_range;
        
                   % define the first point
                   ad_0 = a(1);
        
                   % Define the last point (meaning last point of the time array)
                   a_1 = a(end);
        
                   % Solve
                   b = [ad_0; a_1];
                   syms t;
                   A_1 = @(t)[(1-t).^5; 5.*t.*(1-t).^4; 10.*(t.^2).*(1-t).^3; 10.*(t.^3).*(1-t).^2; 5.*(t.^4).*(1-t).^1; t.^5];
                   A_2 = diff(A_1, t);
                   A_2 = matlabFunction(A_2);
                   A = [A_1(0), A_1(1)]';
                   
                   x0 = [1, 1, 1, 1, 1, 1];
                   A_points = A_1(time);
                   f = @(x) norm(A_points'*[x(1); x(2); x(3); x(4); x(5); x(6)]-a');
                   P=fmincon(f, x0, [], [], A, b);
                  
                   y = sym(A_1)'*P';
                   y = matlabFunction(y);
        
                   y = y(time);
                   
                   diff_bez_norm = sym(A_2)'*P';
                   diff_time = 1/time_range;
                   velocity = diff_bez_norm * diff_time;
                   acceleration = diff(velocity, t)/time_range;
                   velocity = matlabFunction(velocity);
                   acceleration = matlabFunction(acceleration);
                   velocity = velocity(time);
                   acceleration = acceleration(time);
                   if tracking(j) == "x"
                        obj.trajectories(i).u = velocity;
                        obj.trajectories(i).udot = acceleration;
                   elseif tracking(j) == "y"
                       obj.trajectories(i).v = velocity;
                       obj.trajectories(i).vdot = acceleration;
                   else
                       obj.trajectories(i).r = velocity;
                       obj.trajectories(i).rdot = acceleration;
                   end
                   
                   time = time_original;
               end
            end
        end
        
        function obj = filter_u_based_on_udot(obj)
            for i = 1:length(obj.trajectories)
               removeNegativeUDotFilterValue = 0;
                condition_1 = (obj.trajectories(i).udot > removeNegativeUDotFilterValue) & (obj.trajectories(i).encoder_velocity * obj.rw > obj.trajectories(i).u); 
                condition_2 = (obj.trajectories(i).udot < removeNegativeUDotFilterValue) & (obj.trajectories(i).encoder_velocity * obj.rw < obj.trajectories(i).u); 
                filter = condition_1 | condition_2;
                field_names = fieldnames(obj.trajectories(i));
                for j = 1:length(field_names)
                    obj.trajectories(i).(field_names{j}) = obj.trajectories(i).(field_names{j})(filter);                
                end      
            end
        end

        function obj = combine_all_trajectories(obj)
            obj.trajectory.x = [];
            obj.trajectory.y = [];
            obj.trajectory.h = [];
            obj.trajectory.time = [];
            obj.trajectory.u = [];
            obj.trajectory.v = [];
            obj.trajectory.r = [];
            obj.trajectory.udot = [];
            obj.trajectory.vdot = [];
            obj.trajectory.rdot = [];
            obj.trajectory.encoder_velocity = [];
    
            for i = 1:length(obj.trajectories)
                field_names = fieldnames(obj.trajectories(i));
                for j = 1:length(field_names)
                    obj.trajectory.(field_names{j}) = [obj.trajectory.(field_names{j}), obj.trajectories(i).(field_names{j})];                
                end
            end
        end
   end
end