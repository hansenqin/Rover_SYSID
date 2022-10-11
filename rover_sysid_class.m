classdef rover_sysid_class < handle
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
        %% Vehicle constants 
        lf                  % Distance between front axle and center of mass
        lr                  % Distance between rear axle and center of mass
        m                   % Mass 
        Izz                 % Moment of Inertia
        servo_offset        % Servo offset 
        rw                  % Wheel radius
        g                   % Gravity constant
        robot_frame         % Robot tracking frame eg. base_link
        bag                 % Input rosbags


        %% Structs to store signals
        rover_debug_states_out = struct('time', 0, 'x', 0, 'y', 0, 'h', 0, 'u', 0, 'v', 0, 'r', 0, 'w', 0, 'delta_cmd', 0);
        desired_states = struct('time', 0, 'ud', 0, 'vd', 0, 'rd', 0);
        wheel_encoder = struct('time', 0, 'encoder_position', 0);
        vehicle_states_from_slam = struct('time', 0, 'x', 0, 'y', 0, 'h', 0);
        vehicle_states_from_mocap = struct('time', 0, 'x', 0, 'y', 0, 'h', 0);
        trajectory = struct('time', 0, 'x', 0, 'y', 0, 'h', 0, 'encoder_position', 0, 'delta_cmd', 0);
        trajectories = struct('trajectory', 0);

        % signal stuff
        start_time_offset;
        standard_time;  %standard usd to synchronize all signals
          
   end
   
   methods
           function obj = rover_sysid_class(bag_name, rover_config, varargin)
                %% Load rosbag and config json file
                close all;
                obj.bag = rosbag(bag_name);                                 % Read the bag
                rover_config = read_json(obj, rover_config);                % Read the json file with vehicle constants
                
                %% Set vehicle constants
                obj.lf = rover_config.lf;
                obj.lr = rover_config.lr;
                obj.m = rover_config.m;
                obj.Izz = rover_config.Izz;
                obj.servo_offset = rover_config.servo_offset;
                obj.rw = rover_config.rw;
                obj.g = rover_config.g;
                obj.robot_frame = string(rover_config.robot_frame);    
               
                %% Parse optional inputs
                for i = 1:2:length(varargin)                                % Work for a list of name-value pairs
                    if ischar(varargin{i}) || isstring(varargin{i})         % Check if is character
                        obj.(varargin{i}) = varargin{i+1};                  % Override or add parameters to structure.
                    end
                end
                
                %% Load data 
                load_rover_debug_states(obj);
                load_desired_states(obj);
                load_wheel_encoder_data(obj);
                load_vehicle_states_from_slam_data(obj);
                load_vehicle_states_from_mocap_data(obj);

                %% Sync Rate with specified struct
                std_time = obj.vehicle_states_from_mocap.time;
                set_standard_time(obj, std_time);
                structs_to_sync = ["wheel_encoder", "rover_debug_states_out"];
                synchronize_signals_sample_rate(obj, structs_to_sync);

                %% Seperate Trajectories
                tracking_data_from = "vehicle_states_from_mocap";           % vehicle_states_from_slam
                get_trajectories_from_tracking_data(obj);

                %% Fit Bezier Curve 
                fit_bezier_to_trajectories(obj);

                %% Filter Udot values that don't look good
%                 filter_u_based_on_udot(obj);

                %% Combine all trajectories in one struct
                combine_all_trajectories(obj);
                
           end
           
           
           function json = read_json(obj, fname)
                fid = fopen(fname);
                raw = fread(fid,inf); 
                str = char(raw'); 
                fclose(fid); 
                json = jsondecode(str);
           end

           function obj = load_rover_debug_states(obj)
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
                bSel = select(obj.bag,'Topic','/state_out/rover_debug_state_out');
                msgStructs = readMessages(bSel,'DataFormat','struct');
                obj.desired_states.time = cell2mat(cellfun(@(s)cast(s.Header.Stamp.Sec,'double')*1e9+cast(s.Header.Stamp.Nsec,'double'),msgStructs,'uni',0))*1e-9;
                obj.desired_states.ud = cell2mat(cellfun(@(s)s.Ud,msgStructs,'uni',0));
                obj.desired_states.vd = cell2mat(cellfun(@(s)s.Vd,msgStructs,'uni',0));
                obj.desired_states.rd = cell2mat(cellfun(@(s)s.Rd,msgStructs,'uni',0));
           end

           function obj = load_wheel_encoder_data(obj)
                bSel = select(obj.bag,'Topic','/joint_states');
                msgStructs = readMessages(bSel,'DataFormat','struct');
                obj.wheel_encoder.time = cell2mat(cellfun(@(s)cast(s.Header.Stamp.Sec,'double')*1e9+cast(s.Header.Stamp.Nsec,'double'),msgStructs,'uni',0))*1e-9;
                obj.wheel_encoder.encoder_position = cell2mat(cellfun(@(s)s.Position,msgStructs,'uni',0));
                
%                 obj.wheel_encoder.encoder_velocity = (diff(obj.wheel_encoder.encoder_position)./diff(obj.wheel_encoder.time));
%                 obj.wheel_encoder.encoder_velocity  = -obj.wheel_encoder.encoder_velocity*2*pi/(4096);
%                 obj.vehicle_encoder.encoder_velocity = smoothdata(obj.vehicle_encoder.encoder_velocity, 'gaussian', 20);
%                 obj.wheel_encoder.encoder_position = obj.wheel_encoder.encoder_position(1:end-1);
%                 obj.wheel_encoder.time = obj.wheel_encoder.time(1:end-1);
           end

           function obj = load_vehicle_states_from_slam_data(obj)
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

           function obj = synchronize_signals_sample_rate(obj, structs)
                for i = 1:length(structs)
                    field_names = fieldnames(obj.(structs(i)));
                    time = obj.(structs(i)).time;
                    time = time - time(1) + obj.standard_time(1);
                    
                    for j = 2:length(field_names)
                        obj.(structs(i)).(field_names{j}) = interp1(time(:),obj.(structs(i)).(field_names{j}), obj.standard_time(:), 'linear', 'extrap'); 
                    end
                    obj.(structs(i)).time = obj.standard_time(:);
                end
           end
           
           
           function obj = set_standard_time(obj, time)
               obj.standard_time = time;
           end

           function obj = get_trajectories_from_tracking_data(obj)
                %% Compute Temporary Trajectory
               x = obj.vehicle_states_from_mocap.x;
               y = obj.vehicle_states_from_mocap.y;
               h = obj.vehicle_states_from_mocap.h;
               time = obj.vehicle_states_from_mocap.time;
               encoder_position = obj.wheel_encoder.encoder_position;
               delta_cmd = obj.rover_debug_states_out.delta_cmd;
               u = [];
               v = [];
               r = [];
               udot = [];
               vdot = [];
               rdot = [];
    
               for i = 1:length(x)-1
                    u = [u; (x(i+1) - x(i)) / (time(i+1) - time(i) + 0.001)];
                    v = [v; (y(i+1) - y(i)) / (time(i+1) - time(i) + 0.001)];
                    r = [r; (h(i+1) - h(i)) / (time(i+1) - time(i) + 0.001)];
               end
               time = time(1:end-1);
    
               for i = 1:length(u)-1
                    udot = [udot; (u(i+1) - u(i)) / (time(i+1) - time(i) + 0.001)];
                    vdot = [vdot; (v(i+1) - v(i)) / (time(i+1) - time(i) + 0.001)];
                    rdot = [rdot; (r(i+1) - r(i)) / (time(i+1) - time(i) + 0.001)];
               end
               time = time(1:end-1);
               x = x(1:end-2);
               y = y(1:end-2);
               h = h(1:end-2);
               u = u(1:end-1);
               v = v(1:end-1);
               r = r(1:end-1);
               encoder_position = encoder_position(1:end-2);
               delta_cmd = delta_cmd(1:end-2);

               removeNegativeUFilterValue = 0.0;
               removeNegativeU = true;
               if removeNegativeU
                    filter = u > removeNegativeUFilterValue ;
                    x = x(filter);
                    y = y(filter);
                    h = h(filter);
                    encoder_position = encoder_position(filter);
                    delta_cmd = delta_cmd(filter);
               end
                
               obj.trajectories = [];
               start_idx = 1;
               threshHoldDistance = 0.5;
               numOfTrajectories = 0;
               while start_idx <= length(x)-1
                   end_idx = start_idx + 1;
                   for i = start_idx+1:length(x)-1
                       distance = (( x(i) -  x(i+1))^2 + ( y(i) -  y(i+1))^2)^0.5;  
                       if distance > threshHoldDistance
                           end_idx = i;
                           break;
                       end
                   end
                   if (end_idx - start_idx) > 10
                       obj.trajectory.x =  x(start_idx:end_idx)';
                       obj.trajectory.y =  y(start_idx:end_idx)';
                       obj.trajectory.h =  h(start_idx:end_idx)';
                       obj.trajectory.time =  time(start_idx:end_idx)';
                       obj.trajectory.encoder_position  = encoder_position (start_idx:end_idx)';
                       obj.trajectory.delta_cmd = delta_cmd(start_idx:end_idx)';
                       obj.trajectories = [obj.trajectories, obj.trajectory];
                       numOfTrajectories = numOfTrajectories + 1;
                   end
                   start_idx = end_idx + 1;
               end
                
               disp([numOfTrajectories, " trajectories were detected"]);
           end

           function obj = fit_bezier_to_trajectories(obj)
               tracking = ["x", "y", "h", "encoder_position"];
               for i = 1:length(obj.trajectories)
               
               % fit bezier for each x and y in the trajectory
               % note that a=x or a=y depending on what you are calculating
               % return y, a function that you can call as y(t), where t is the
               % time vector. P is also the coefficients of the bezier function
               for j = 1:length(tracking)
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
                   elseif tracking(j) == "h"
                       obj.trajectories(i).r = velocity;
                       obj.trajectories(i).rdot = acceleration;
                   else
                       obj.trajectories(i).encoder_position = y;
                       obj.trajectories(i).encoder_velocity = -velocity*2*pi/(4096);
                   end
                   
                   time = time_original;
               end
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
            delta = obj.trajectory.delta_cmd;
  
            % Calculate the slip ratio
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

            % Select and fit a curve to the data in the linear region 
            lambdaSelected = lambda(abs(lambda)<0.01);
            F_xSelected = F_x(abs(lambda)<0.01);
            
            % Longitudinal linear curve through (0,0) using fmincon
            x0 = 10;
            f = @(x) norm(obj.m.*obj.g.*lambdaSelected'.*x - F_xSelected);
            xLinearCoef = fmincon(f,x0);
       end
    
       function plot_longitudnal_linear_tire_curve(obj, xLinearCoef, lambda, F_x)
            % Longitudinal
            close all;
            figure(1);
            scatter(-lambda+0.085, F_x);
            hold on;
            xLinear = @(t) obj.m*obj.g*obj.lf*xLinearCoef(1)*t/(obj.lf+obj.lr);
            t = -0.2:0.01:0.2;
            plot(t, xLinear(t), "LineWidth", 2);
            xlabel("Front Slip Ratio");
            ylabel("Front Longitudinal Tire Force (N)");
            xlim([-0.5,0.5]);
            ylim([-15 15]);
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
            ylim([-20, 20]);
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
            obj.trajectory.delta_cmd = [];
            
    
            for i = 1:length(obj.trajectories)
                field_names = fieldnames(obj.trajectories(i));
                for j = 1:length(field_names)
                    obj.trajectory.(field_names{j}) = [obj.trajectory.(field_names{j}), obj.trajectories(i).(field_names{j})];                
                end
            end
        end
   end
end