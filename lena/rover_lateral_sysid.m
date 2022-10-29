classdef rover_lateral_sysid < handle
   properties
        %% Vehicle constants 
        rover_config      % Vehicle constants
        bag               % Input rosbag
        bag_name          % Bag name

        %% Signal standardization
        start_time_offset;                                                  % TODO: use this or delete it?
        standard_time;                                                      % standard usd to synchronize all signals

        %% Structs to store signals
        rover_debug_states_out = struct('time', 0, 'x', 0, 'y', 0, ...
            'h', 0, 'u', 0, 'v', 0, 'r', 0, 'w', 0, 'delta_cmd', 0);
        desired_states = struct('time', 0, 'ud', 0, 'vd', 0, 'rd', 0);
        wheel_encoder = ...
            struct('time', 0, 'encoder_position', 0, 'encoder_velocity', 0);
        vehicle_states_from_slam = ...
            struct('time', 0, 'x', 0, 'y', 0, 'h', 0);
        vehicle_states_from_mocap = ...
            struct('time', 0, 'x', 0, 'y', 0, 'h', 0);
        vehicle_delta_command = struct('time', 0, 'delta_cmd', 0);
        vehicle_motor_current_command = ...
            struct('time', 0, 'motor_current', 0);

        trajectory = struct('time', 0, 'x', 0, 'y', 0, 'h', 0, ...
            'fitted_x', 0, 'fitted_y', 0, 'encoder_position', 0, ...
            'delta_cmd', 0, 'num_diff_u', 0, 'num_diff_v', 0, ...
            'num_diff_r', 0, 'encoder_velocity', 0);
        trajectories = struct('trajectory', 0);
        states = struct('time', 0, 'x', 0, 'y', 0, 'h', 0, 'u', 0, ...
            'v', 0, 'r', 0, 'udot', 0, 'vdot', 0, 'rdot', 0);
   end
   methods
           %% Driver
           function obj = rover_lateral_sysid(bag_name, rover_config, varargin)
                % Load rosbag and vehicle constants from config json file
                close all;
                obj.bag_name = bag_name;
                obj.bag = rosbag(bag_name);                                 % Read the bag
                obj.rover_config = read_json(rover_config);                     % Read the json file with vehicle constants
                obj.rover_config.robot_frame = string(obj.rover_config.robot_frame);
               
                % Parse optional inputs
                for i = 1:2:length(varargin)                                % Work for a list of name-value pairs
                    if ischar(varargin{i}) || isstring(varargin{i})         % Check if is character
                        obj.(varargin{i}) = varargin{i+1};                  % Override or add parameters to structure.
                    end
                end
                
                % Load data 
                load_rover_debug_states(obj);
                load_desired_states(obj);
                load_wheel_encoder_data(obj);
                load_vehicle_states_from_slam_data(obj);
                load_vehicle_states_from_mocap_data(obj);

                % Sync Rate with specified struct
                structs_to_sync = struct(...
                    "rover_debug_states_out", obj.rover_debug_states_out, ...
                    "desired_states", obj.desired_states, ...
                    "wheel_encoder", obj.wheel_encoder, ...
...                     "vehicle_states_from_slam", obj.vehicle_states_from_slam, ...
                    "vehicle_states_from_mocap", obj.vehicle_states_from_mocap, ...
                    "vehicle_delta_command", obj.vehicle_delta_command, ...
                    "vehicle_motor_current_command", obj.vehicle_motor_current_command);

                % Make all of the signals start at 0
                structs_to_sync = synchronize_signals_start_time(structs_to_sync);

                % Remove sections where the h from mocap resets from +pi to
                % -pi
                max_yaw = 3.1;
                condition_mocap = abs(structs_to_sync.vehicle_states_from_mocap.h) < max_yaw;
                structs_to_sync = remove_via_condition(structs_to_sync.vehicle_states_from_mocap, structs_to_sync, condition_mocap);

                % Remove undesirable sections of the trajectory
                minimum_u = 0.1;
                structs_to_sync = remove_negative_u(structs_to_sync.vehicle_states_from_mocap, structs_to_sync, minimum_u);
                condition_encoder = structs_to_sync.wheel_encoder.encoder_velocity > minimum_u;
                structs_to_sync = remove_via_condition(structs_to_sync.wheel_encoder, structs_to_sync, condition_encoder);

                % Interpolate so that the time vectors match
                reference_struct = structs_to_sync.vehicle_states_from_mocap;
                structs_to_sync = synchronize_signals_sample_rate(reference_struct, structs_to_sync);

                % Seperate Trajectories


                % Fit Bezier Curve 


                % Plotting
                [fLinearCoef, rLinearCoef, alphaf, alphar, F_yf, F_yr] ...
                    = compute_lateral_tire_curve_coefficients ...
                    (obj.rover_config, obj.states);
                plot_lateral_linear_tire_curve(fLinearCoef, ...
                    rLinearCoef, alphaf, alphar, F_yf, F_yr)
           end
           
           %% Load data from bag
           function obj = load_rover_debug_states(obj)
               % Loads states from state estimator: time, x, y, h, u, v, r, w,
               % delta_cmd
                bSel = select(obj.bag,'Topic','/state_out/rover_debug_state_out');
                msgStructs = readMessages(bSel,'DataFormat','struct');
                obj.rover_debug_states_out.time = cell2mat(cellfun(@(s)cast(s.Header.Stamp.Sec,'double')*1e9+cast(s.Header.Stamp.Nsec,'double'),msgStructs,'uni',0))*1e-9;
                obj.rover_debug_states_out.x = cell2mat(cellfun(@(s)s.X,msgStructs,'uni',0));
                obj.rover_debug_states_out.y = cell2mat(cellfun(@(s)s.Y,msgStructs,'uni',0));
                obj.rover_debug_states_out.h = cell2mat(cellfun(@(s)s.H,msgStructs,'uni',0));
                obj.rover_debug_states_out.u = cell2mat(cellfun(@(s)s.U,msgStructs,'uni',0));
                obj.rover_debug_states_out.v = cell2mat(cellfun(@(s)s.V,msgStructs,'uni',0));
                obj.rover_debug_states_out.r = cell2mat(cellfun(@(s)s.R,msgStructs,'uni',0));
                obj.rover_debug_states_out.w = cell2mat(cellfun(@(s)s.W,msgStructs,'uni',0));
                obj.rover_debug_states_out.delta_cmd = cell2mat(cellfun(@(s)s.DeltaCmd,msgStructs,'uni',0));
           end
           function obj = load_desired_states(obj)
               % Loads the desired states: time, ud, vd, rd
                bSel = select(obj.bag,'Topic','/state_out/rover_debug_state_out');
                msgStructs = readMessages(bSel,'DataFormat','struct');
                obj.desired_states.time = cell2mat(cellfun(@(s)cast(s.Header.Stamp.Sec,'double')*1e9+cast(s.Header.Stamp.Nsec,'double'),msgStructs,'uni',0))*1e-9;
                obj.desired_states.ud = cell2mat(cellfun(@(s)s.Ud,msgStructs,'uni',0));
                obj.desired_states.vd = cell2mat(cellfun(@(s)s.Vd,msgStructs,'uni',0));
                obj.desired_states.rd = cell2mat(cellfun(@(s)s.Rd,msgStructs,'uni',0));
           end
           function obj = load_wheel_encoder_data(obj)
               % Loads states from the wheel encoder: time,
               % encoder_position, encoder_velocity
                bSel = select(obj.bag,'Topic','/joint_states');
                msgStructs = readMessages(bSel,'DataFormat','struct');
                obj.wheel_encoder.time = cell2mat(cellfun(@(s)cast(s.Header.Stamp.Sec,'double')*1e9+cast(s.Header.Stamp.Nsec,'double'),msgStructs,'uni',0))*1e-9;
                obj.wheel_encoder.encoder_position = cell2mat(cellfun(@(s)s.Position,msgStructs,'uni',0));
                
                obj.wheel_encoder.encoder_velocity = (diff(obj.wheel_encoder.encoder_position)./diff(obj.wheel_encoder.time));
                obj.wheel_encoder.encoder_velocity  = -obj.wheel_encoder.encoder_velocity*2*pi/(4096);
%                 obj.wheel_encoder.encoder_velocity = smoothdata(obj.wheel_encoder.encoder_velocity, 'gaussian', 20);
                obj.wheel_encoder.encoder_position = obj.wheel_encoder.encoder_position(1:end-1);
                obj.wheel_encoder.time = obj.wheel_encoder.time(1:end-1);
           end
           function obj = load_vehicle_states_from_slam_data(obj)
               % Loads in data from slam: time, x, y, h
                bSel = select(obj.bag, 'Topic', '/tf');
                tf = cell2mat(readMessages(bSel, 'DataFormat', 'struct'));
                obj.vehicle_states_from_slam.time = [];
                obj.vehicle_states_from_slam .x = [];
                obj.vehicle_states_from_slam .y = [];
                obj.vehicle_states_from_slam .h = [];
                for i = 1:size(tf)
                    if size(tf(i).Transforms, 2) > 0
                        if tf(i).Transforms.Header.FrameId == "map" && tf(i).Transforms.ChildFrameId == obj.rover_config.robot_frame
                            time = (cast(tf(i, 1).Transforms.Header.Stamp.Sec, 'double')*1e9 + cast(tf(i, 1).Transforms.Header.Stamp.Nsec, 'double'))*1e-9;
                            obj.vehicle_states_from_slam .time = [obj.vehicle_states_from_slam.time; time];
                            obj.vehicle_states_from_slam .x = [obj.vehicle_states_from_slam.x; tf(i).Transforms.Transform.Translation.X];
                            obj.vehicle_states_from_slam .y = [obj.vehicle_states_from_slam.y; tf(i).Transforms.Transform.Translation.Y];
                            q = [tf(i).Transforms.Transform.Rotation.W, tf(i).Transforms.Transform.Rotation.X, tf(i).Transforms.Transform.Rotation.Y, tf(i).Transforms.Transform.Rotation.Z];
                            [z, y, x] = quat2angle(q);
                            obj.vehicle_states_from_slam.h = [obj.vehicle_states_from_slam .h; z];
                        end
                    end
                end
           end
           function obj = load_vehicle_states_from_mocap_data(obj)
                % Loads states from mocap: time, x, y, h

                bSel = select(obj.bag, 'Topic', '/mocap');
                mocap = cell2mat(readMessages(bSel, 'DataFormat', 'struct'));

                time = zeros(length(mocap), 1);
                x = zeros(length(mocap), 1);
                y = zeros(length(mocap), 1);
                h = zeros(length(mocap), 1);

                for i = 1:size(mocap)
                    x(i) = mocap(i).Pose.Position.X*1e-3;
                    y(i) = -mocap(i).Pose.Position.Z*1e-3;
                    time(i) = (cast(mocap(i, 1).Header.Stamp.Sec, 'double')*1e9 + cast(mocap(i, 1).Header.Stamp.Nsec, 'double'))*1e-9;

                    q = [mocap(i).Pose.Orientation.W, mocap(i).Pose.Orientation.X, mocap(i).Pose.Orientation.Y, mocap(i).Pose.Orientation.Z];
                    [x_, y_, z_] = quat2angle(q, 'XYZ');
                    h(i) = z_;  
                end

                obj.vehicle_states_from_mocap.time = time;
                obj.vehicle_states_from_mocap.x = x;
                obj.vehicle_states_from_mocap.y = y;
                obj.vehicle_states_from_mocap.h = -h;

           end
           function obj = load_commands(obj)
               % Loads in data from the vesc motor controller (time, 
               % motor_current) and the steering angle servo (time,
               % delta_cmd)
               bSel_servo = select(obj.bag,'Topic','/vesc/sensors/servo_position_command');
               bSel_motor_current = select(obj.bag,'Topic','/vesc/sensors/core');
               msgStructs_servo = readMessages(bSel_servo,'DataFormat','struct');
               msgStructs_motor_current = readMessages(bSel_motor_current,'DataFormat','struct');

               % A -1 is multiplied witbh delta_cmd due to the set up of the
               % mechanical steering mechanism that inverts the direction of
               % the servo with the direction of the wheels.
               obj.vehicle_delta_command.time = table2array(bSel_servo.MessageList(:,1));
               obj.vehicle_delta_command.delta_cmd = -cell2mat(cellfun(@(s)s.Data,msgStructs_servo,'uni',0));
               obj.vehicle_motor_current_command.time = table2array(bSel_motor_current.MessageList(:,1));
               obj.vehicle_motor_current_command.motor_current = cell2mat(cellfun(@(s)s.State.CurrentMotor,msgStructs_motor_current,'uni',0));
           end

           %% Bezier data fitting
           
   end
end