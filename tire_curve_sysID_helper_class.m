classdef tire_curve_sysID_helper_class
    %Authors: Hansen Qin, Lena Trang
    %Email: qinh@umich.edu
    %Created: 5/17/2022
    %Modified: 5/17/2022
    %
    %Purpose: solve for the lateral tire characetristic curve coefficients
    %         given a ros bag with vehicle states and vehicle commands.
    %         The tire characterisitc curve is in the form:
    %         F_lateral = C1*tanh(C2*slip_angle)
    
    
    
   properties
        imu_Fs
        pass_band
        lf
        lr
        m
        Izz
        
        vehicle_states = struct(time, x, y, h, u, v, r, u_dot, v_dot, r_dot)     
        vehicle_commands = struct(time, delta_cmd, motor_current);
        desired_states = struct(time, ud, vd, rd);
        
        %signal stuff
        start_time_offset;
        auto_flag_data;
        standard_time;  %standard usd to synchronize all signals
        
   end
   
   methods
       function obj = tire_curve_sysID_helper_class(bag)
        %Constrctor
            set_standard_time(obj, bag)
           
            load_vehicle_states_data(obj,bag);
            load_zonotope_data(obj,bag);
            load_imu_data(obj,bag);
            load_param_gen_data(obj,bag);
            
            synchronize_signals_sample_rate()
            
            
       end
      
       
       function obj = set_standard_time(obj, bag)
           bSel = select(bag,'Topic','/state_out/rover_debug_state_out');
           msgStructs = readMessages(bSel,'DataFormat','struct');
           obj.standard_time = cell2mat(cellfun(@(s)cast(s.Header.Stamp.Sec,'double')*1e9+cast(s.Header.Stamp.Nsec,'double'),msgStructs,'uni',0))*1e-9;
       end
       
       
       function obj = synchronize_signals_sample_rate(obj, varargin)
        % [z1,z2,...] = synchronizeSignalsSampleRate(tdes,t1,z1,t2,z2,...,interp_type)
        %
        % Given some desired sample times and any number of time vectors (1-by-Nt)
        % and associated trajectories (Nstates-by-Nt), resample the trajectories
        % linearly at the desired times.
            
            varargout = cell(1,nargout);

            out_idx = 1 ;
            for i = 1:length(varargin)
                field_names = fieldnames(varargin{i});
                time = varargin{i}.time;
                for j = 2:length(field_names)
                    Z = varargin{i}.field_names{j};
                    if length(time)==1 && obj.standard_time==time
                        varargout{out_idx} = Z;
                    else
                        varargout{out_idx} = interp1(time(:),Z',obj.standard_time(:),'linear')';
                    end
                    out_idx = out_idx + 1 ;
                end
            end
       end
       
       
       function synchronize_signals_time(obj)
           %Might not be needed
           
           
       end
       
       
       function obj = load_vehicle_states_data(obj, bag)
            bSel = select(bag,'Topic','/state_out/rover_debug_state_out');
            msgStructs = readMessages(bSel,'DataFormat','struct');
            time = cell2mat(cellfun(@(s)cast(s.Header.Stamp.Sec,'double')*1e9+cast(s.Header.Stamp.Nsec,'double'),msgStructs,'uni',0))*1e-9;
            obj.vehicle_states.x = cell2mat(cellfun(@(s)s.X,msgStructs,'uni',0));
            obj.vehicle_states.y = cell2mat(cellfun(@(s)s.Y,msgStructs,'uni',0));
            obj.vehicle_states.h = cell2mat(cellfun(@(s)s.H,msgStructs,'uni',0));
            obj.vehicle_states.u = cell2mat(cellfun(@(s)s.U,msgStructs,'uni',0));
            obj.vehicle_states.v = cell2mat(cellfun(@(s)s.V,msgStructs,'uni',0));
            obj.vehicle_states.r = cell2mat(cellfun(@(s)s.R,msgStructs,'uni',0));
            obj.desired_states.ud = cell2mat(cellfun(@(s)s.Ud,msgStructs,'uni',0));
            obj.desired_states.vd = cell2mat(cellfun(@(s)s.Vd,msgStructs,'uni',0));
            obj.desired_states.rd = cell2mat(cellfun(@(s)s.Rd,msgStructs,'uni',0));
            obj.auto_flag_data = cell2mat(cellfun(@(s)s.RunningAuto,msgStructs,'uni',0));
            
       end
       
       
       function load_commands(obj)
            bSel_servo = select(bag_file,'Topic','/vesc/sensors/servo_position_command');
            bSel_motor_current = select(bag_file,'Topic','/vesc/sensors/core');
            msgStructs_servo = readMessages(bSel_servo,'DataFormat','struct');
            msgStructs_motor_current = readMessages(bSel_motor_current,'DataFormat','struct');
            
            obj.vehicle_commands.time = table2array(bSel_servo.MessageList(:,1))*1e9;
            obj.vehicle_commands.delta_cmd = cell2mat(cellfun(@(s)s.Data,msgStructs_servo,'uni',0));
            obj.vehicle_commands.motor_current = cell2mat(cellfun(@(s)s.State.CurrentMotor,msgStructs_motor_current,'uni',0));
            
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
       
       
       function get_tire_curve_coefficients(obj)
           
       end
       
       
       function lowpass_signals(obj, cutoff_frequency)
    
       end
       
   end
end