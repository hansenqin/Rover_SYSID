classdef tire_curve_sysID_helper_class
   properties
        imu_Fs
        pass_band
        pass_band_imu
        pass_band_acc
        lf
        lr
        m
        Izz
        
        vehicle_states = struct(x, y, h, u, v, r, u_dot, v_dot, r_dot)     
        vehicle_commands = struct(delta_cmd,motor_cmd);
        desired_states = struct(ud, vd, rd);
        
        %signal stuff
        start_time_offset;
        auto_flag_data;
        standard_time;  %standard usd to synchronize all signals
        
   end
   
   methods
       function obj = tire_curve_sysID_helper_class()
        %Constrctor
            obj.imu_Fs = 35;
            obj.pass_band = 0.5;
            obj.lf = 0.20;
            obj.lr = 0.115;
            obj.m = 4.956;
            obj.Izz = 0.1;
       end
       
       
       function load_ros_bags(obj,bag)
            
            set_standard_time(obj, bag)
           
            load_states_data(obj,bag);
            load_zonotope_data(obj,bag);
            load_imu_data(obj,bag);
            load_param_gen_data(obj,bag);
            
       end
      
       
       function obj = set_standard_time(obj, bag)
           bSel = select(bag,'Topic','/state_out/rover_debug_state_out');
           msgStructs = readMessages(bSel,'DataFormat','struct');
           obj.standard_time = cell2mat(cellfun(@(s)cast(s.Header.Stamp.Sec,'double')*1e9+cast(s.Header.Stamp.Nsec,'double'),msgStructs,'uni',0))*1e-9;
       end
       
       
       function [varargout] = synchronize_signals_sample_rate(obj, tdes, varargin)
        % [z1,z2,...] = synchronizeSignalsSampleRate(tdes,t1,z1,t2,z2,...,interp_type)
        %
        % Given some desired sample times and any number of time vectors (1-by-Nt)
        % and associated trajectories (Nstates-by-Nt), resample the trajectories
        % linearly at the desired times.

            varargout = cell(1,nargout) ;

            if ischar(varargin{end})
                interp_type = varargin{end} ;
            else
                interp_type = 'linear' ;
            end

            out_idx = 1 ;
            for idx = 1:2:(length(varargin)-1)
                t = varargin{idx} ;
                Z = varargin{idx+1} ;
                if length(t)==1 && tdes==t
                    varargout{out_idx} = Z;
                else
                    varargout{out_idx} = interp1(t(:),Z',tdes(:),interp_type)' ;
                end

                out_idx = out_idx + 1 ;
            end
       end
       
       
       function synchronize_signals_time(obj)
           %Might not be needed
           
           
       end
       
       
       function obj = load_states_data(obj, bag)
            bSel = select(bag,'Topic','/state_out/rover_debug_state_out');
            msgStructs = readMessages(bSel,'DataFormat','struct');
            t = cell2mat(cellfun(@(s)cast(s.Header.Stamp.Sec,'double')*1e9+cast(s.Header.Stamp.Nsec,'double'),msgStructs,'uni',0))*1e-9;
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
       
       
       function load_param_gen_data(obj)
           %TODO
       end
       
   end
end