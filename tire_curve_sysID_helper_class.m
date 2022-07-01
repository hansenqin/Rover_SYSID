classdef tire_curve_sysID_helper_class < handle
    %Authors: Hansen Qin, Lena Trang
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
        
        %rosobj.bag
        bag
        
        % structs that store signals 
        vehicle_states = struct('time', 0, 'x', 0, 'y', 0, 'h', 0, 'u', 0, 'v', 0, 'r', 0, 'w', 0);
        vehicle_delta_command = struct('time', 0, 'delta_cmd', 0);
        vehicle_motor_current_command = struct('time', 0, 'motor_current', 0);
        desired_states = struct('time', 0, 'ud', 0, 'vd', 0, 'rd', 0);
        auto_flag_data = struct('time', 0, 'auto_flag', 0);
        
        
        % signal stuff
        start_time_offset;
        standard_time;  %standard usd to synchronize all signals
        
        
        % auto flags on
        auto_flag_on
        
        
   end
   
   methods
       function obj = tire_curve_sysID_helper_class(bag_name, rover_config, varargin)
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
            
            % set default flags
            obj.auto_flag_on = 0;
           
            % parse optional intputs
            for i = 1:2:length(varargin) % work for a list of name-value pairs
                if ischar(varargin{i}) || isstring(varargin{i}) % check if is character
                    obj.(varargin{i}) = varargin{i+1}; % override or add parameters to structure.
                end
            end
            
            %Load data
            load_vehicle_states_data(obj);
            load_zonotope_data(obj);
            load_commands(obj)
            load_imu_data(obj);
            load_param_gen_data(obj);
            load_auto_flag(obj)
            
            % Sync signals
            % reference_topic is used as the standard sampling
            % rate, and all other topics will have their sample rates
            % synchronized to the standard sampling rate
            reference_topic = '/state_out/rover_debug_state_out';
            set_standard_time(obj, reference_topic)
            structs_to_sync = ["vehicle_states", "vehicle_delta_command", "vehicle_motor_current_command", "desired_states", "auto_flag_data"];
            synchronize_signals_sample_rate(obj, structs_to_sync);
            
            % Low pass signals
            structs_to_low_pass = ["vehicle_states"];
            lowpass_signals(obj, structs_to_low_pass)
            
            % Auto filter signals 
            if obj.auto_flag_on
                structs_to_auto_filter = ["vehicle_states", "vehicle_delta_command", "vehicle_motor_current_command", "desired_states"];
                auto_filter(obj, structs_to_auto_filter)
            end
            
       end
       
       
       function json = read_json(obj, fname)
            fid = fopen(fname);
            raw = fread(fid,inf); 
            str = char(raw'); 
            fclose(fid); 
            json = jsondecode(str);
       end
       
       
       function obj = set_standard_time(obj, reference_topic)
          
           bSel = select(obj.bag,'Topic',reference_topic);
           msgStructs = readMessages(bSel,'DataFormat','struct');
           obj.standard_time = cell2mat(cellfun(@(s)cast(s.Header.Stamp.Sec,'double')*1e9+cast(s.Header.Stamp.Nsec,'double'),msgStructs,'uni',0))*1e-9;
       end
       
       
       function obj = synchronize_signals_sample_rate(obj, structs)
            
            for i = 1:length(structs)
                field_names = fieldnames(obj.(structs(i)));
                time = obj.(structs(i)).time;
                for j = 2:length(field_names)
                    obj.(structs(i)).(field_names{j}) = interp1(time(:),obj.(structs(i)).(field_names{j}),obj.standard_time(:),'linear')';
                end
                obj.(structs(i)).time = obj.standard_time(:);
            end
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
            
            obj.desired_states.time = cell2mat(cellfun(@(s)cast(s.Header.Stamp.Sec,'double')*1e9+cast(s.Header.Stamp.Nsec,'double'),msgStructs,'uni',0))*1e-9;
            obj.desired_states.ud = cell2mat(cellfun(@(s)s.Ud,msgStructs,'uni',0));
            obj.desired_states.vd = cell2mat(cellfun(@(s)s.Vd,msgStructs,'uni',0));
            obj.desired_states.rd = cell2mat(cellfun(@(s)s.Rd,msgStructs,'uni',0));
            
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
            obj.vehicle_delta_command.delta_cmd = -cell2mat(cellfun(@(s)s.Data,msgStructs_servo,'uni',0));
            obj.vehicle_delta_command.time = table2array(bSel_servo.MessageList(:,1));
            
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
       

       function [fLinearCoef, rLinearCoef, fNonlinearCoef, rNonlinearCoef, alphaf, alphar, F_ywf, F_ywr] = get_tire_curve_coefficients(obj)
            % Original function header: function get_tire_curve_coefficients(obj)

            vdot = [];
            for k=1:1:length(obj.vehicle_states.v)-1
                newvdot = (obj.vehicle_states.v(k+1)-obj.vehicle_states.v(k))/(obj.vehicle_states.time(k+1)-obj.vehicle_states.time(k));
                vdot = [vdot newvdot];
            end
            rdot = [];
            for k=1:1:length(obj.vehicle_states.r)-1
                newrdot = (obj.vehicle_states.r(k+1)-obj.vehicle_states.r(k))/(obj.vehicle_states.time(k+1)-obj.vehicle_states.time(k));
                rdot = [rdot newrdot];
            end
            
            % Adjust size of other vectors
            
            Fx_est = 0.4*obj.vehicle_motor_current_command.motor_current - 6.0897*tanh(10.5921*obj.vehicle_states.u);
            F_xr = Fx_est(1:end-1);
            F_xfw = Fx_est(1:end-1);
            
            x = obj.vehicle_states.x(1:end-1);
            y = obj.vehicle_states.y(1:end-1);
            h = obj.vehicle_states.h(1:end-1);
            u = obj.vehicle_states.u(1:end-1);
            v = obj.vehicle_states.v(1:end-1);
            r = obj.vehicle_states.r(1:end-1);
            delta=obj.vehicle_delta_command.delta_cmd(1:end-1)+obj.servo_offset;
            % Find vf and vr
            v_f = v+obj.lf.*r;
            v_r = v-obj.lr.*r;
            % Rotate to wheel frame
            u_wf = u.*cos(-delta)-v_f.*sin(-delta);
            v_wf = u.*sin(-delta)+v_f.*cos(-delta);
            u_wr = u;
            v_wr = v_r;
            % Find slip angles and lateral forces
            alphaf = - atan(v_wf./sqrt((u_wf).^2 +0.05));
            alphar = - atan(v_wr./sqrt((u_wr).^2 +0.05));
            F_ywf=(obj.lr.*obj.m.*(vdot+u.*r)+rdot*obj.Izz-(obj.lf+obj.lr)*sin(delta).*F_xfw)./((obj.lf+obj.lr).*cos(delta));
            F_ywr=(obj.m*(vdot+u.*r)-rdot*obj.Izz/obj.lf)./(1+obj.lr/obj.lf);
            % Sort the data
            [alphafSorted, If] = sort(alphaf);
            F_ywfSorted = F_ywf(If);
            [alpharSorted, Ir] = sort(alphar);
            F_ywrSorted = F_ywr(Ir);
            % Select and fit a curve to the data in the linear region 
            alphafSelected = alphafSorted(abs(alphafSorted)<0.2);
            F_ywfSelected = F_ywfSorted(abs(alphafSorted)<0.2);
%             fLinearCoef=polyfit(alphafSelected, F_ywfSelected, 1);
            alpharSelected = alpharSorted(abs(alpharSorted)<0.2);
            F_ywrSelected = F_ywrSorted(abs(alpharSorted)<0.2);
%             rLinearCoef = polyfit(alpharSelected, F_ywrSelected, 1);

            % RANSAC 
            [fLinearCoef, fLinearCoef_pre]=obj.linear_ransac_fmincon(alphafSelected, F_ywfSelected, 1, .5);
            [rLinearCoef, rLinearCoef_pre]=obj.linear_ransac_fmincon(alpharSelected, F_ywrSelected, 2, 0.75);
            
            % Nonlinear curve using fmincon
            x0 = [1 1];
            f = @(x) norm(x(1)*tanh(x(2)*alphaf)-F_ywf);
            fNonlinearCoef = fmincon(f,x0);
            r = @(x) norm(x(1)*tanh(x(2)*alphar)-F_ywr);
            rNonlinearCoef = fmincon(r,x0);


            %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             t = [-0.1:0.01:0.1]
%             coef = polyfit(alphafSelected, F_ywfSelected, 1)
%             f = t.*coef(1) + coef(2);
%             figure(1);
%             plot(t, f);
%             hold on;
%             f = t.*fLinearCoef_pre;
%             plot(t, f);
%             legend('poly', 'ransac_pre')
%             
%             coef = polyfit(alpharSelected, F_ywrSelected, 1)
%             f = t.*coef(1) + coef(2);
%             figure(2);
%             plot(t, f);
%             hold on;
%             f = t.*rLinearCoef_pre;
%             plot(t, f);
%             legend('poly', 'ransac_pre')
       end

       function [linearCoef] = linear_ransac(obj, alpha, F_y, figure_num, max_dist)
           % Purpose: find coefficients for the linear model using ransac
           % and polyfit
           sampleSize = 2;
           points = [alpha', F_y'];
            fitLineFcn = @(points) polyfit(points(:,1),points(:,2),1); % fit function using polyfit
            evalLineFcn = ...   % distance evaluation function
              @(model, points) sum((points(:, 2) - polyval(model, points(:,1))).^2,2);  
            [modelRANSAC, inlierIdx] = ransac(points,fitLineFcn,evalLineFcn, ...
              sampleSize,max_dist);
            linearCoef = polyfit(points(inlierIdx,1),points(inlierIdx,2),1);
       end


       function [linearCoef, linearCoef_pre] = linear_ransac_fmincon(obj, alpha, F_y, figure_num, max_dist)
            % Purpose: find coefficients for the linear model using ransac
            % and fmincon
            sampleSize = 2;
            points = [alpha', F_y'];
            x0 = [1];
            f = @(x) norm(x(1)*alpha-F_y);
            linearCoef_pre = fmincon(f, x0);
            fitLineFcn = @(points) fmincon(f,x0); % fit function using fmincon
            evalLineFcn = ...   % distance evaluation function
              @(model, points) sum((points(:, 2) - model*points(:,1)).^2,2);  
            [modelRANSAC, inlierIdx] = ransac(points,fitLineFcn,evalLineFcn, ...
              sampleSize,max_dist);
            f = @(x) norm(x(1)*points(inlierIdx,1)-points(inlierIdx,2));
            linearCoef = fmincon(f,x0);
       end

       function plot_linear_tire_curve(obj, fLinearCoef, rLinearCoef, alphaf, alphar, F_ywf, F_ywr)
           % Purpose: 
            % Front tire
            figure(1);
            scatter(alphaf, F_ywf);
            hold on;
            fLinear = @(t) fLinearCoef.*t;
            t = -0.1:0.01:0.1;
            plot(t, fLinear(t), "LineWidth", 2);
            xlabel("Front Slip Angle");
            ylabel("Front Lateral Tire Force (N)");
            title("Fywf="+round(fLinearCoef,2)+"*alphaf")
            hold off;
            
            % Rear tire
            figure(2);
            scatter(alphar, F_ywr);
            hold on;
            rLinear = @(t) rLinearCoef*t;
            t = -0.1:0.01:0.1;
            plot(t, rLinear(t), "LineWidth", 2);
            xlabel("Rear Slip Angle");
            ylabel("Rear Lateral Tire Force (N)");
            title("Fywr="+round(rLinearCoef,2)+"*alpha")
            hold off;
       end
       
       function [delta_u] = low_speed_sys_id(obj)
            % Purpose: find the longitudinal speed state error (u dot)
            % This is used for data where there are is no turning manuvers,
            % only speed change manuvers of speeds < 0.5 m/s
            % Note: consider doing realignment for the time of the
            % estimated because the effects of the motor current come into 
            % affect only after some time has elapsed

            % udot = (Fxwf + Fxwr)/m+vr
            % delta_u = A_predicted - A_SLAM
            
            % A_predicted
            Fx_est = 0.4*obj.vehicle_motor_current_command.motor_current - 6.0897*tanh(10.5921*obj.vehicle_states.u);
            A_predicted = Fx_est/obj.m;
            A_predicted(end) = []; % match the data with the actual

            % A_actual
            v = obj.vehicle_states.v;
            v(end)=[];
            r = obj.vehicle_states.r;
            r(end)=[];
            vr = v.*r;
             A_actual = diff(obj.vehicle_states.u)./diff(obj.vehicle_states.time') - vr;

            % Difference delta u
            delta_u = A_predicted - A_actual;

            %%%%%%%%%%%%%%%%%%%%% 2022-06-30-15-33-35
            % Plot u dot(t)
            time = obj.vehicle_states.time-1.6566176*10^9;
            time(end) = [];
            %%%%%%%%%%%%%%%%%%%%% 2022-06-30-15-33-35
            % Choosing time period
            start_time = 0;
            end_time = 325;            
            A_predicted = A_predicted((start_time<time)&(time<end_time));
            A_actual = A_actual((start_time<time)&(time<end_time));
            delta_u = delta_u((start_time<time)&(time<end_time));
            time = time((start_time<time)&(time<end_time));

            % Remove trajectories with SLAM issues % trajectory number, speed
            [time, A_predicted, A_actual, delta_u] = obj.remove_trajectory(time, A_predicted, A_actual, delta_u, 52, 57); % 2, 2.0
            [time, A_predicted, A_actual, delta_u] = obj.remove_trajectory(time, A_predicted, A_actual, delta_u, 145, 150); % 6, 1.7
            [time, A_predicted, A_actual, delta_u] = obj.remove_trajectory(time, A_predicted, A_actual, delta_u, 158, 162); % 5, 1.6
            [time, A_predicted, A_actual, delta_u] = obj.remove_trajectory(time, A_predicted, A_actual, delta_u, 195, 200); % 10, 1.3
            [time, A_predicted, A_actual, delta_u] = obj.remove_trajectory(time, A_predicted, A_actual, delta_u, 230, 234); % 13, 1.0
            %%%%%%%%%%%%%%%%%%%%%

            figure(1);
%             plot(time, A_predicted);
%             hold on; 
%             plot(time, A_SLAM);
%             plot(time, delta_u);
            scatter(time, A_predicted);
            hold on; 
            scatter(time, A_actual);
            scatter(time, delta_u);

            xlim([0 325]);
            ylim([-3 3]);
            xlabel("time")
            ylabel("u dot")
            legend("estimated", "actual", "error")

            hold off;
       end

       function [u, ud, time] = test_print(obj)
           % To see whether or not the rosbag recorded correctly

           % Import data
           u = obj.vehicle_states.u;
           time = obj.vehicle_states.time-1.6566176*10^9;
           ud = obj.desired_states.ud;
           w = obj.vehicle_states.w;
           %%%%%%%%%%%%%%%%%%%%%%%%%%%%
           % Adjustments for 2022-06-30-15-33-35           
           start_time = 0;
           end_time = 325;
           u = u((start_time<time)&(time<end_time));
           ud = ud((start_time<time)&(time<end_time));
           time = time((start_time<time)&(time<end_time));
           w = w((start_time<time)&(time<end_time));
           %%%%%%%%%%%%%%%%%%%%%%%%%%%%

           % Plot u wrt time
           figure(2);
           plot(time, u);
           hold on;
           plot(time, ud);
           plot(time, w);
           xlabel('time');
           ylabel('u')
           xlim([0 325]); % 2022-06-30-15-33-35
           hold off;

           % Scatter u wrt to time
           figure(3)
           scatter(time, u);
           hold on;
           scatter(time, ud);
           legend('u','ud');
           xlabel('time');
           ylabel('u')
           xlim([0 325])
       end
       
       function [time, A_predicted, A_actual, delta_u] = remove_trajectory(obj, time, A_predicted, A_actual, delta_u, start_time, end_time)
            % Remove trajectories with SLAM issues
            A_predicted((time>start_time)&(time<end_time)) = [];
            A_actual((time>start_time)&(time<end_time)) = [];
            delta_u((time>start_time)&(time<end_time)) = [];
            time((time>start_time)&(time<end_time)) = [];
       end


    
   end
end