classdef rover_lateral_sysid_class < handle
   
    properties
        %% Vehicle constants 
        lf                % Distance between front axle and center of mass
        lr                % Distance between rear axle and center of mass
        m                 % Mass 
        Izz               % Moment of Inertia
        servo_offset      % Servo offset 
        rw                % Wheel radius
        g                 % Gravity constant
        robot_frame       % Robot tracking frame eg. base_link
        bag               % Input rosbags
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
        bezier_curve = struct('time', 0, 'x', 0, 'y', 0, ...
            'y_fitted_on_x', 0, 'dy_dx', 0, 'velocity_x_global', 0, ...
            'velocity_y_global', 0, 'velocity_longitudinal', 0, ...
            'velocity_lateral', 0);
        
        %% Stored for plotting
        lambdas             % Lambdas for all trajectories
        F_xs                % F_xs for all trajectories
   end
   
   methods
           %% Driver
           function obj = rover_lateral_sysid_class(bag_name, rover_config, varargin)
                % Load rosbag and vehicle constants from config json file
                close all;
                obj.bag_name = bag_name;
                obj.bag = rosbag(bag_name);                                 % Read the bag
                rover_config = read_json(obj, rover_config);                % Read the json file with vehicle constants
                set_vehicle_constants(obj, rover_config);                   % Set the constants given the rover configuration
               
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
                std_time = obj.vehicle_states_from_mocap.time;
                set_standard_time(obj, std_time);
                structs_to_sync = ["wheel_encoder", "rover_debug_states_out"];
                synchronize_signals_sample_rate(obj, structs_to_sync);

                % Seperate Trajectories
                get_trajectories_from_tracking_data(obj);

                % Fit Bezier Curve 
                fit_bezier_to_trajectories(obj);

                % Filter Udot values that don't look good
                filter_u_based_on_udot(obj);

                % Combine all trajectories in one struct
                combine_all_trajectories(obj);
                
           end
           
           %% Load constants from json file
           function json = read_json(obj, fname)
               % Opens and reads the json file
                fid = fopen(fname);
                raw = fread(fid,inf); 
                str = char(raw'); 
                fclose(fid); 
                json = jsondecode(str);
           end
           function obj = set_vehicle_constants(obj, rover_config)
               % Sets the rover constants based on the rover_config
                obj.lf = rover_config.lf;
                obj.lr = rover_config.lr;
                obj.m = rover_config.m;
                obj.Izz = rover_config.Izz;
                obj.servo_offset = rover_config.servo_offset;
                obj.rw = rover_config.rw;
                obj.g = rover_config.g;
                obj.robot_frame = string(rover_config.robot_frame);   
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
                obj.rover_debug_states_out .delta_cmd = cell2mat(cellfun(@(s)s.DeltaCmd,msgStructs,'uni',0));
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
                        if tf(i).Transforms.Header.FrameId == "map" && tf(i).Transforms.ChildFrameId == obj.robot_frame
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
                obj.vehicle_states_from_mocap.time = [];
                obj.vehicle_states_from_mocap.x = [];
                obj.vehicle_states_from_mocap.y = [];
                obj.vehicle_states_from_mocap.h = [];
                
                time = [];
                x = [];
                y = [];
                h = [];

                bSel = select(obj.bag, 'Topic', '/mocap');
                mocap = cell2mat(readMessages(bSel, 'DataFormat', 'struct'));

                for i = 1:size(mocap)
                    x = [x; mocap(i).Pose.Position.X*1e-3];
                    y = [y; -mocap(i).Pose.Position.Z*1e-3];
                    t = (cast(mocap(i, 1).Header.Stamp.Sec, 'double')*1e9 + cast(mocap(i, 1).Header.Stamp.Nsec, 'double'))*1e-9;
                    time = [time; t];

                    q = [mocap(i).Pose.Orientation.W, mocap(i).Pose.Orientation.X, mocap(i).Pose.Orientation.Y, mocap(i).Pose.Orientation.Z];
                    [z_, y_, x_] = quat2angle(q);
                    h = [h; z_];           
                end

                obj.vehicle_states_from_mocap.time = time;
                obj.vehicle_states_from_mocap.x = x;
                obj.vehicle_states_from_mocap.y = y;
                obj.vehicle_states_from_mocap.h = h;

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

           %% Adjust the data to the standard time
           function obj = set_standard_time(obj, time)
               % Sets the time that all other structs will be synchonized
               % with
               obj.standard_time = time;
           end
           % fix
           function obj = synchronize_signals_sample_rate(obj, structs)
               % Interpolates the given structs to be at the same time
               % as the standard time. TODO: change this to be after the
               % bezier fitting
               for i = 1:length(structs)
                    field_names = fieldnames(obj.(structs(i)));
                    time = obj.(structs(i)).time;
                    % TODO: change this to something else? because the time
                    % vec may not always be the same
                    time = time - time(1) + obj.standard_time(1);
                    for j = 2:length(field_names)
                        obj.(structs(i)).(field_names{j}) = interp1(time(:),obj.(structs(i)).(field_names{j}), obj.standard_time(:), 'linear', 'extrap'); 
                    end
                    obj.(structs(i)).time = obj.standard_time(:);
                end
           end
           % fix
           function obj = get_trajectories_from_tracking_data(obj)
               % Divides the data into trajectories. Also removes the time
               % when u is negative or very small
                %% Compute Temporary Trajectory
               x = obj.vehicle_states_from_mocap.x;
               y = obj.vehicle_states_from_mocap.y;
               h = obj.vehicle_states_from_mocap.h;
               time = obj.vehicle_states_from_mocap.time;
               encoder_position = obj.wheel_encoder.encoder_position;
               delta_cmd = obj.rover_debug_states_out.delta_cmd;

               numOfTrajectories = 0;
               obj.trajectories = [];
               if obj.bag_name == "mocap_lateral_10_09_1.bag"
                   start_idx = 200;
                   end_idx = 3800;
                   obj.trajectory.x =  x(start_idx:end_idx)';
                   obj.trajectory.y =  y(start_idx:end_idx)';
                   obj.trajectory.h =  h(start_idx:end_idx)';
                   obj.trajectory.time =  time(start_idx:end_idx)';
                   obj.trajectory.encoder_position  = encoder_position (start_idx:end_idx)';
                   obj.trajectory.delta_cmd = delta_cmd(start_idx:end_idx)';
                   obj.trajectories = [obj.trajectories, obj.trajectory];
                   numOfTrajectories = numOfTrajectories + 1;
                   
                   start_idx = 4800;
                   end_idx = 7408;
                   obj.trajectory.x =  x(start_idx:end_idx)';
                   obj.trajectory.y =  y(start_idx:end_idx)';
                   obj.trajectory.h =  h(start_idx:end_idx)';
                   obj.trajectory.time =  time(start_idx:end_idx)';
                   obj.trajectory.encoder_position  = encoder_position (start_idx:end_idx)';
                   obj.trajectory.delta_cmd = delta_cmd(start_idx:end_idx)';
                   obj.trajectories = [obj.trajectories, obj.trajectory];
                   numOfTrajectories = numOfTrajectories + 1;
               elseif obj.bag_name == "mocap_lateral_10_09_2.bag" 
                   start_idx = 200;
                   end_idx = 1500;
                   obj.trajectory.x =  x(start_idx:end_idx)';
                   obj.trajectory.y =  y(start_idx:end_idx)';
                   obj.trajectory.h =  h(start_idx:end_idx)';
                   obj.trajectory.time =  time(start_idx:end_idx)';
                   obj.trajectory.encoder_position  = encoder_position (start_idx:end_idx)';
                   obj.trajectory.delta_cmd = delta_cmd(start_idx:end_idx)';
                   obj.trajectories = [obj.trajectories, obj.trajectory];
                   numOfTrajectories = numOfTrajectories + 1;
                   
                   start_idx = 1501;
                   end_idx = 4200;
                   obj.trajectory.x =  x(start_idx:end_idx)';
                   obj.trajectory.y =  y(start_idx:end_idx)';
                   obj.trajectory.h =  h(start_idx:end_idx)';
                   obj.trajectory.time =  time(start_idx:end_idx)';
                   obj.trajectory.encoder_position  = encoder_position (start_idx:end_idx)';
                   obj.trajectory.delta_cmd = delta_cmd(start_idx:end_idx)';
                   obj.trajectories = [obj.trajectories, obj.trajectory];
                   numOfTrajectories = numOfTrajectories + 1;
               else
                   throw("Invalid bag"); 
               end
                
               disp([numOfTrajectories, " trajectories were detected"]);
           end

           % Data fitting
           function obj = fit_bezier_to_trajectories(obj)
               for i = 1:length(obj.trajectories)
                   %% Load variables in x, y, time for bezier curve fitting
                   x = obj.trajectories(i).x;
                   y = obj.trajectories(i).y;
                   encoder_position = obj.trajectories(i).encoder_position;
                   time = obj.trajectories(i).time;

                   %% Assign x_0, y_0 based on the trajectory and bag file
                   if obj.bag_name == "mocap_lateral_10_09_1.bag" 
                       if i == 1
                            x_0 = 0.5;
                            y_0 = 1.0;
                       elseif i == 2
                            x_0 = -0.5;
                            y_0 = 0.5;
                       end
                   elseif obj.bag_name == "mocap_lateral_10_09_2.bag" 
                       if i == 1
                            x_0 = 0.5;
                            y_0 = 0.0;
                       elseif i == 2
                            x_0 = 0.5;
                            y_0 = 0.5;
                       end
                   end

                   r = sqrt((x - x_0).^2 + (y - y_0).^2);
%                    theta = atan((y - y_0) ./ (x - x_0));
                   theta = [];

                   for i = 1:length(x)
                        bias = 0;
                        dy = y(i) - y_0;
                        dx = x(i) - x_0;

                        if (dx < 0)
                            bias = pi;
                        elseif (dx > 0 && dy < 0)
                            bias = 2*pi;
                        end
                        theta = [theta, atan(dy / dx) + bias];
                   end
                   
                   %% Fit Bezier curves on (time, r), (time, theta), (time, x), (time, y), (x, y), (vx, vy), (time, encoder_position)
                   x_cart_check = r .* cos(theta) + x_0;
                   y_cart_check = r .* sin(theta) + y_0;

                   [fitted_r, fr, dr_global, d2r_global] =    fit_bezier_curve(obj, time, r);
                   [fitted_theta, ftheta, dtheta_global, d2theta_global] =    fit_bezier_curve(obj, time, theta);
                   [fitted_encoder_position, f_encoder_position, encoder_velocity, enocoder_acc] =    fit_bezier_curve(obj, time, encoder_position);

                   fitted_x = fitted_r .* cos(fitted_theta) + x_0;
                   fitted_y = fitted_r .* sin(fitted_theta) + y_0;

                   vx_global = cos(fitted_theta) .* dr_global - fitted_r .* sin(fitted_theta) .* dtheta_global;
                   vy_global = sin(fitted_theta) .* dr_global + fitted_r .* cos(fitted_theta) .* dtheta_global;

                   %% Needs to be corrected use the above two eqautions and take derivative wrt time (Missing the last dtheta_global term)

                   ax_global = d2r_global*cos(fitted_theta)...
                       -2*dr_global*dtheta_global*sin(fitted_theta)...
                       -fitted_r*dtheta_global*dtheta_global*cos(fitted_theta)...
                       -fitted_r*d2theta_global*sin(fitted_theta);
                   ay_global = d2r_global*sin(fitted_theta)...
                       +2*dr_global*dtheta_global*cos(fitted_theta)...
                       -fitted_r*dtheta_global*dtheta_global*sin(fitted_theta)...
                       +fitted_r.*d2theta_global*cos(fitted_theta);

                   [fitted_y_over_x, f_x, slope_of_f_x, d2_f_x] =  fit_bezier_curve(obj, fitted_x, fitted_y);             

                   theta = atan2(fitted_y(end)-fitted_y(1), fitted_x(end)-fitted_x(1));
                   Rot = [cos(theta),sin(theta);-sin(theta),cos(theta)];
                   xy_local = Rot * ([fitted_x(:)'; fitted_y(:)'] - [fitted_x(1);fitted_y(1)]);
                   %% Get magnitude of v_global and a_global
%                    v_global = sqrt(vx_global.^2 + vy_global.^2);
%                    a_global = sqrt(ax_global.^2 + ay_global.^2);

                   %% Find theta of vectors from the atan(y/x)                   
                   theta_v_global = atan(vy_global./vx_global);
                   theta_a_global = atan(ay_global./ax_global);

                   %% Get theta from the slope of fitted function                   
                   theta_over_f_x = atan(slope_of_f_x);
                   v_loc = [vx_global;vx_global];
                   a_loc = [vx_global;vx_global];
                   dthv = theta_over_f_x + theta_v_global*0;
                   dtha = theta_over_f_x + theta_a_global*0;
                   
                   for j = 1:length(ax_global)
                       v_loc(:,j) = [cos(dthv(j)),sin(dthv(j));-sin(dthv(j)),cos(dthv(j))] * [vx_global(j); vy_global(j)];
                       a_loc(:,j) = [cos(dtha(j)),sin(dtha(j));-sin(dtha(j)),cos(dtha(j))] * [ax_global(j); ay_global(j)];
                   end

                   %% WRONG NEED TO CHANGE TO RIGID BODY TRANSFORM Get Longitdunal and Lateral componenets of the vectors
        

%                    v_longitudnal = cos(theta_v_global - theta_over_f_x) .* v_global;
%                    v_lateral = sin(theta_v_global - theta_over_f_x) .* v_global;
% 
%                    a_longitudnal = cos(theta_a_global - theta_over_f_x) .* a_global;
%                    a_lateral = sin(theta_a_global - theta_over_f_x) .* a_global;

                   %% Save the values in the trajectories struct

                   obj.trajectories(i).fitted_x = fitted_x;
                   obj.trajectories(i).u = v_loc(1, :);
                   obj.trajectories(i).udot = a_loc(1, :);

                   obj.trajectories(i).fitted_y = fitted_y;
                   obj.trajectories(i).v = v_loc(2, :);
                   obj.trajectories(i).vdot = a_loc(2, :);

                   [fitted_theta, ftheta, yaw_global, yawdotglobal] = fit_bezier_curve(obj, time, theta_over_f_x);

%                    obj.trajectories(i).fitted_h = fitted_theta;
                   obj.trajectories(i).r = yaw_global;
                   obj.trajectories(i).rdot = yawdotglobal;

                   obj.trajectories(i).encoder_position = fitted_encoder_position;
                   obj.trajectories(i).encoder_velocity = -encoder_velocity*2*pi/(4096);
                   %                    obj.trajectories(i).num_diff_encoder_velocity = num_diff_encoder_velocity;
               end
           end
           function [fitted_y, f_of_x, first_derivative, second_derivative] =   fit_bezier_curve(obj, x, y)
               a = y;
               time = x;
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
               f_of_x = matlabFunction(y);

               fitted_y = f_of_x(time);

               diff_bez_norm = sym(A_2)'*P';
               diff_time = 1/time_range;
               velocity = diff_bez_norm * diff_time;
               acceleration = diff(velocity, t)/time_range;
               velocity = matlabFunction(velocity);
               acceleration = matlabFunction(acceleration);
               first_derivative = velocity(time);
               second_derivative = acceleration(time);
           end
           function obj = filter_u_based_on_udot(obj)
               for i = 1:length(obj.trajectories)
                   threshold = 1;
                   filter = abs(obj.trajectories(i).u - obj.trajectories(i).encoder_velocity * obj.rw) < threshold;
                   %                 condition_1 = (obj.trajectories(i).udot > removeNegativeUDotFilterValue) & (obj.trajectories(i).encoder_velocity * obj.rw > obj.trajectories(i).u);
                   %                 condition_2 = (obj.trajectories(i).udot < removeNegativeUDotFilterValue) & (obj.trajectories(i).encoder_velocity * obj.rw < obj.trajectories(i).u);
                   %                 filter = condition_1 | condition_2;
                   field_names = fieldnames(obj.trajectories(i));
                   for j = 1:length(field_names)
                       obj.trajectories(i).(field_names{j}) = obj.trajectories(i).(field_names{j})(filter);
                   end
               end
           end

           function obj = combine_all_trajectories(obj)
               % Adds all the trajectories in obj.trajectories to
               % obj.trajectory
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
               obj.trajectory.delta_cmd = [];

               for i = 1:length(obj.trajectories)
                   field_names = fieldnames(obj.trajectories(i));
                   for j = 1:length(field_names)
                       obj.trajectory.(field_names{j}) = [obj.trajectory.(field_names{j}), obj.trajectories(i).(field_names{j})];
                   end
               end
           end

           function [xLinearCoef, lambda, F_x] = get_longitudnal_tire_curve_coefficients(obj)
               % Calculates the slip ratio, longitudinal force, and fitted
               % linear curve for the longitudinal tire curve. Returns
               % coefficents of the linear fitting, slip ratio, and forces.
               %% Loading data
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
               delta = obj.trajectory.delta_cmd;

               %% Calculate the slip ratio
               lambda = [];
               lambda_numerator = obj.rw.*w-u;
               for i = 1:length(udot)
                   if udot(i) < 0
                       lambda = [lambda, lambda_numerator(i)./sqrt(u(i).^2+0.0001)];
                   else
                       lambda = [lambda, lambda_numerator(i)./sqrt((obj.rw.*w(i)).^2+0.0001)];
                   end
               end
               % Find longitudinal force: wheel encoder method
               F_x = obj.m.*(udot-v.*r);

               %% Filter F_x and lambdas to remove noise
               lambda = -lambda;
               removeNoise = true;
               if (removeNoise)
                   filter_top_left = ((lambda <= -0.2) & (F_x >= -6.6)) | ((lambda <= -0.18) & (F_x >= -5.85)) ...
                       | ((lambda <= -0.13) & (F_x >= -4.3)) | ((lambda <= -0.11) & (F_x >= -3.16)) ...
                       | ((lambda <= -0.07) & (F_x >= -0.34)) | ((lambda <= 0.0) & (F_x >= 6.8)) ;
                   filter_top = (F_x >= 8.2);
                   filter_bottom = (F_x <= -9.6);
                   filter_bottom_left = ((lambda <= -0.2) & (F_x <= -9.6)) | ((lambda <= -0.14) & (F_x <= -10)) ...
                       | ((lambda >= -0.18) & (lambda <= -0.08) & (F_x <= -7.26)) ;
                   filter_bottom_right = ((lambda >= 0.0) & (F_x <= 2.7)) | ((lambda >= 0.0) & (F_x <= 4.13)) ...
                       | ((lambda >= -0.13) & (lambda <= -0.07) & (F_x <= -5.9)) ;
                   filter_top_right = ((lambda >= 0.015) & (F_x <= 5.32));
                   %                 filter_center = ((lambda >= -0.05) & (lambda <= 0) & (F_x <= 0)) ;
                   %                 filter_right_circle = sqrt((lambda - 0.11).^2 + (F_x + 2.86).^2) <= 0.2;
                   filter_center_line = ((lambda >= -0.15) & (lambda <= 0.02) & (F_x <= 64 * lambda + 2.5)) ;
                   filter = filter_center_line | filter_top_left | filter_top | filter_bottom_left | filter_bottom | filter_bottom_right | filter_top_right ;
                   filter = ~filter;
                   lambda = lambda(filter);
                   F_x = F_x(filter);

                   lambda = lambda + 0.055;
               end

               % Select and fit a curve to the data in the linear region
               threshold = 0.03;
               lambdaSelected = lambda(abs(lambda) < threshold);
               F_xSelected = F_x(abs(lambda) < threshold);

               % Longitudinal linear curve through (0,0) using fmincon
               x0 = 10;
               f = @(x) norm(obj.m.*obj.g.*lambdaSelected'.*x - F_xSelected);
               xLinearCoef = fmincon(f,x0);

           end
           
           %% PLOTTING
           function plot_tracking_data(obj)
                % Misc. plotting for testing/debug
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
           function plot_longitudnal_linear_tire_curve(obj, xLinearCoef, lambda, F_x)
               % Plots the linear longitudinal tire curve
               figure(1);
               clf;
               hold on;
               fxvslambda = scatter(lambda, F_x);
               fxvslambda.Annotation.LegendInformation.IconDisplayStyle = 'off';
               xLinear = @(t) obj.m*obj.g*xLinearCoef(1)*t;
               t = -0.2:0.01:0.2;
               plot(t, xLinear(t), "LineWidth", 2, 'DisplayName', "Slope of Surface Adhesion Coefficient");
               xlabel("Slip Ratio");
               ylabel("Longitudinal Tire Force (N)");
               xlim([-0.5,0.5]);
               legend;
               ylim([-15 15]);
               hold off;
           end
           function plot_lateral_linear_tire_curve(obj, yLinearCoef, alpha, F_y, position)
               % Plots the front or rear lateral curve depending on the
               % arguments. Note that for for front, position==1 and for rear,
               % position==0.
               if position % Front tire
                   figure(2);
                   clf;
                   xlabel("Front Slip Angle");
                   ylabel("Front Lateral Tire Force (N)");
                   title("Fywf="+round(yLinearCoef(1),2)+"*alpha_f")
               else % Rear tire
                   figure(3);
                   clf;
                   xlabel("Rear Slip Angle");
                   ylabel("Rear Lateral Tire Force (N)");
                   title("Fywr="+round(rLinearCoef(1),2)+"*alpha_r");
               end
               % Plot the data and the fitted curve
               scatter(alpha, F_y);
               hold on;
               yLinear = @(t) yLinearCoef(1)*t;
               t = -0.2:0.01:0.2;
               plot(t, yLinear(t), "LineWidth", 2);
               xlim([-0.3,0.3]);
               hold off;
           end
           function plot_linear_tire_curve(obj, xLinearCoef, fLinearCoef, rLinearCoef, lambda, alphaf, alphar, F_x, F_yf, F_yr)
               % Plots the linear longitudinal and lateral curves
               plot_longitudnal_linear_tire_curve(obj, xLinearCoef, lambda, F_x);
               plot_lateral_linear_tire_curve(obj, fLinearCoef, alphaf, F_yf, 1);
               plot_lateral_linear_tire_curve(obj, rLinearCoef, alphar, F_yr, 0);
           end
   end
end