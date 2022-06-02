classdef tire_curve_sysID_helper_class < handle
    %Authors: Hansen Qin, Lena Trang
    %Email: qinh@umich.edu
    %Created: 5/17/2022
    %Modified: 6/2/2022
    %
    %Purpose: solve for the lateral tire characetristic curve coefficients
    %         given a ros bag with vehicle states and vehicle commands.
    %         The tire characterisitc curve is in the form:
    %         F_lateral = C1*tanh(C2*slip_angle)
    
    
    
   properties
        pass_band
        lf
        lr
        m
        Izz
        servo_offset
        
        % structs that store signals 
        vehicle_states = struct('time', 0, 'x', 0, 'y', 0, 'h', 0, 'u', 0, 'v', 0, 'r', 0)
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
       function obj = tire_curve_sysID_helper_class(bag, varargin)
            % set vehicle constants
            obj.pass_band = 50;
            obj.lf = 0.20;
            obj.lr = 0.115;
            obj.m = 4.956;%kg
            obj.Izz = 0.1;
            obj.servo_offset = 0.5;
            obj.auto_flag_on = 0;
           
            % set default auto_flag to "OFF"
            for i = 1:2:length(varargin) % work for a list of name-value pairs
                if ischar(varargin{i}) || isstring(varargin{i}) % check if is character
                    obj.(varargin{i}) = varargin{i+1}; % override or add parameters to structure.
                end
            end
            
            
            %Load data
            load_vehicle_states_data(obj,bag);
            load_zonotope_data(obj,bag);
            load_commands(obj,bag)
            load_imu_data(obj,bag);
            load_param_gen_data(obj,bag);
            load_auto_flag(obj,bag)
            
            
            % Sync signals
            % reference_topic is used as the standard sampling
            % rate, and all other topics will have their sample rates
            % synchronized to the standard sampling rate
            reference_topic = '/state_out/rover_debug_state_out';
            set_standard_time(obj, bag, reference_topic)
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
       
       
       function obj = set_standard_time(obj, bag, reference_topic)
          
           bSel = select(bag,'Topic',reference_topic);
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
       
       
       function obj = load_vehicle_states_data(obj, bag)
            bSel = select(bag,'Topic','/state_out/rover_debug_state_out');
            msgStructs = readMessages(bSel,'DataFormat','struct');
            obj.vehicle_states.time = cell2mat(cellfun(@(s)cast(s.Header.Stamp.Sec,'double')*1e9+cast(s.Header.Stamp.Nsec,'double'),msgStructs,'uni',0))*1e-9;
            obj.vehicle_states.x = cell2mat(cellfun(@(s)s.X,msgStructs,'uni',0));
            obj.vehicle_states.y = cell2mat(cellfun(@(s)s.Y,msgStructs,'uni',0));
            obj.vehicle_states.h = cell2mat(cellfun(@(s)s.H,msgStructs,'uni',0));
            obj.vehicle_states.u = cell2mat(cellfun(@(s)s.U,msgStructs,'uni',0));
            obj.vehicle_states.v = cell2mat(cellfun(@(s)s.V,msgStructs,'uni',0));
            obj.vehicle_states.r = cell2mat(cellfun(@(s)s.R,msgStructs,'uni',0));
            
            obj.desired_states.time = cell2mat(cellfun(@(s)cast(s.Header.Stamp.Sec,'double')*1e9+cast(s.Header.Stamp.Nsec,'double'),msgStructs,'uni',0))*1e-9;
            obj.desired_states.ud = cell2mat(cellfun(@(s)s.Ud,msgStructs,'uni',0));
            obj.desired_states.vd = cell2mat(cellfun(@(s)s.Vd,msgStructs,'uni',0));
            obj.desired_states.rd = cell2mat(cellfun(@(s)s.Rd,msgStructs,'uni',0));
            
       end
       
       
       function load_auto_flag(obj,bag)
            bSel = select(bag,'Topic','/state_out/rover_debug_state_out');
            msgStructs = readMessages(bSel,'DataFormat','struct');
            
            obj.auto_flag_data.auto_flag = double(cell2mat(cellfun(@(s)s.RunningAuto,msgStructs,'uni',0)));
            obj.auto_flag_data.time = cell2mat(cellfun(@(s)cast(s.Header.Stamp.Sec,'double')*1e9+cast(s.Header.Stamp.Nsec,'double'),msgStructs,'uni',0))*1e-9;
            obj.start_time_offset = obj.auto_flag_data.auto_flag(1,1);
            
       end
       
       
       function load_commands(obj,bag)
            bSel_servo = select(bag,'Topic','/vesc/sensors/servo_position_command');
            bSel_motor_current = select(bag,'Topic','/vesc/sensors/core');
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

       
       function auto_filter(obj, structs)
           % Only keeps the data from the automatic trajectory running
           for i = 1:length(structs) 
                field_names = fieldnames(obj.(structs(i)));
                for j = 1:length(field_names)
                    signal = obj.(structs(i)).(field_names{j});
                    obj.(structs(i)).(field_names{j}) = signal(logical(obj.auto_flag_data.auto_flag));
                end
           end
       end
       
       
       function load_param_gen_data(obj,bag)
           %TODO
       end
       
       
       function load_zonotope_data(obj,bag)
           %TODO
       end
       
       
       function load_imu_data(obj,bag)
           %TODO
       end
       

       function [fLinearCoef, rLinearCoef, fNonlinearCoef, rNonlinearCoef] = get_tire_curve_coefficients(obj)
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
            F_yr=(obj.m*(vdot+u.*r)-rdot*obj.Izz/obj.lf)./(1+obj.lr/obj.lf);
            % Sort the data
            [alphafSorted, If] = sort(alphaf);
            F_ywfSorted = F_ywf(If);
            [alpharSorted, Ir] = sort(alphar);
            F_yrSorted = F_yr(Ir);
            % Select and fit a curve to the data in the linear region 
            alphafSelected = alphafSorted(abs(alphafSorted)<0.1);
            F_ywfSelected = F_ywfSorted(abs(alphafSorted)<0.1);
            fLinearCoef = polyfit(alphafSelected, F_ywfSelected, 1);
            alpharSelected = alpharSorted(abs(alpharSorted)<0.1);
            F_yrSelected = F_yrSorted(abs(alpharSorted)<0.1);
            rLinearCoef = polyfit(alpharSelected, F_yrSelected, 1);
            % Nonlinear curve using fmincon
            x0 = [1 1];
            f = @(x) norm(x(1)*tanh(x(2)*alphaf)-F_ywf);
            fNonlinearCoef = fmincon(f,x0);
            r = @(x) norm(x(1)*tanh(x(2)*alphar)-F_yr);
            rNonlinearCoef = fmincon(r,x0);
            
            
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
       
   end
end