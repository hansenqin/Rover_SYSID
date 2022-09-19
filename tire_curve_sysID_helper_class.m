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
        rw
        g
        
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
            obj.rw = rover_config.rw;
            obj.g = rover_config.g;
            
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
            set_standard_time(obj, reference_topic);
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
                    obj.(structs(i)).(field_names{j}) = interp1(time(:),obj.(structs(i)).(field_names{j}),obj.standard_time(:),'linear','extrap');
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
       

       function [xLinearCoef, fLinearCoef, rLinearCoef, lambda, alphaf, alphar, F_x, F_yf, F_yr] = get_tire_curve_coefficients(obj)

            % TODO: add curve fitting to this derivative step
            udot = [];
            for k=1:1:length(obj.vehicle_states.u)-1
                newvdot = (obj.vehicle_states.u(k+1)-obj.vehicle_states.u(k))/(obj.vehicle_states.time(k+1)-obj.vehicle_states.time(k));
                udot = [udot newvdot];
            end
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
            
            udot = udot';
            vdot = vdot';
            rdot = rdot';

            x = obj.vehicle_states.x(1:end-1);
            y = obj.vehicle_states.y(1:end-1);
            h = obj.vehicle_states.h(1:end-1);
            u = obj.vehicle_states.u(1:end-1);
            v = obj.vehicle_states.v(1:end-1);
            r = obj.vehicle_states.r(1:end-1);
            w = obj.vehicle_states.w(1:end-1);
            delta=obj.vehicle_delta_command.delta_cmd(1:end-1)+obj.servo_offset;
        
            % Calculate the longitudinal force
            % Current method
%             Fx_est = 0.4*obj.vehicle_motor_current_command.motor_current - 6.0897*tanh(10.5921*obj.vehicle_states.u);
%             F_xr = Fx_est(1:end-1);
%             F_xfw = Fx_est(1:end-1);

            % Calculate the slip ratio
            lambda = [];
            lambda_numerator = obj.rw.*w-u;
            for i = 1:length(lambda_numerator)
                if lambda_numerator < 0
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
            % Nonlinear curve using fmincon
%             x0 = [1 1];
%             f = @(x) norm(x(1)*tanh(x(2)*alphaf)-F_yf);
%             fNonlinearCoef = fmincon(f,x0);
%             r = @(x) norm(x(1)*tanh(x(2)*alphar)-F_yr);
%             rNonlinearCoef = fmincon(r,x0);
       end

       function plot_linear_tire_curve(obj, xLinearCoef, fLinearCoef, rLinearCoef, lambda, alphaf, alphar, F_x, F_yf, F_yr)
            % Longitudinal
            figure(1);
            scatter(lambda, F_x);
            hold on;
            xLinear = @(t) xLinearCoef(1)*t;
            t = -0.2:0.01:0.2;
            plot(t, xLinear(t), "LineWidth", 2);
            xlabel("Front Slip Ratio");
            ylabel("Front Longitudinal Tire Force (N)");
            title("Fxf+Fxr=mgl_r/l"+round(fLinearCoef(1),2)+"*lambda")
            xlim([-1,1]);
            hold off;

            % Front tire lateral
            figure(2);
            scatter(alphaf, F_yf);
            hold on;
            fLinear = @(t) fLinearCoef(1)*t;
            t = -0.2:0.01:0.2;
            plot(t, fLinear(t), "LineWidth", 2);
            xlabel("Front Slip Angle");
            ylabel("Front Lateral Tire Force (N)");
            title("Fywf="+round(fLinearCoef(1),2)+"*alpha")
            xlim([-0.3,0.3]);
            hold off;
            
            % Rear tire lateral
            figure(3);
            scatter(alphar, F_yr);
            hold on;
            rLinear = @(t) rLinearCoef(1)*t;
            t = -0.2:0.01:0.2;
            plot(t, rLinear(t), "LineWidth", 2);
            xlabel("Rear Slip Angle");
            ylabel("Rear Lateral Tire Force (N)");
            title("Fywr="+round(rLinearCoef(1),2)+"*alpha");
            xlim([-0.3,0.3]);
            hold off;
       end
   end
end