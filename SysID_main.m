clear; close all; 


%% load the data
bag_file = rosbag('new_nov.bag');
[gen_param_out_data,...
 parameter_data,...
 polygons_data_time,...
 polygons_data,...
 state_data,...
 rd_data,...
 ud_data,...
 maneuver_type_data,...
 auto_flag_data,...
 mocap_data,...
 sensor_data,...
 motor_cmd,...
 delta_cmd,...
 imu_data,...
 estimator_data,...
 slam_h] = load_data_from_bag(bag_file);

consts = load('rover_const');

%%

mocap_Fs = 35;
imu_Fs = 35;
pass_band = 0.5;
pass_band_imu =0.5;
pass_band_acc =1;
lf = 0.20;
lr = 0.115;
m = 4.956;%kg
Izz = 0.1;
model_type = 5;
plot_figure_1 = 0;
prediction_horizon = 1.5;
plot_animation =  0;


%% Data preprocessing
% 
% for i = 1:100
T = end_time;%slam_txy(end,1)-slam_txy(1,1);% 30;
N = floor(T/prediction_horizon-1);
i_ini = floor(start_time/prediction_horizon)+1;



%new delta
delta_cmd(:,2) =(delta_cmd(:,2) - 0.50)/0.8290*0.84;
start_time_offset = auto_flag_data(1,1)*1e-9;
end_time = auto_flag_data(end,1)*1e-9 -start_time_offset;
mocap_data = mocap_data(1:end,:);
imu_data = imu_data(1:3:end,:);
slam_txy = state_data(1:1:end,1:3);
slam_h   = state_data(1:1:end,4);
ud = state_data(1:1:end,8);
vd = state_data(1:1:end,9);
rd = state_data(1:1:end,10);


mocap_x = mocap_data(:,7);
mocap_y = mocap_data(:,5);
mocap_h = [];

for i = 1:length(mocap_data)
    
    quat = [mocap_data(i,8) mocap_data(i,9) mocap_data(i,10) mocap_data(i,11)];
    rot1 = [1  0  0;
            0  0 -1;
            0  1  0]; %Initial car rotation
    
    roth = rot1'*quat2rotm(quat)*rot1; 
    h_temp = rotm2eul(roth);
    
    mocap_h = [mocap_h h_temp(1)];
end



mocap_h = -[zeros(1,34) mocap_h(1:end-34)];
mocap_x = -[zeros(34,1); mocap_x(1:end-34)/1000];
mocap_y = -[zeros(34,1); mocap_y(1:end-34)/1000];
mocap_txy = [mocap_data(:,1) mocap_x mocap_y];


RR=rotmat(-mocap_h);
diff_mocap_states = diff(mocap_txy);


mocap_spd_vec = diff_mocap_states(:,2:3) ./  diff_mocap_states(:,1);
mocap_spd_vec = [mocap_spd_vec(1,:);mocap_spd_vec];


%obtain u v r
mocap_spd_one_col =  mocap_spd_vec';
mocap_spd_one_col =  mocap_spd_one_col(:);


mocap_uv = RR * mocap_spd_one_col;
mocap_uv = reshape(mocap_uv,2,[])';
mocap_uv_original = mocap_uv;
mocap_uv = lowpass(mocap_uv,pass_band ,mocap_Fs);
mocap_u = mocap_uv(:,1);
mocap_v = mocap_uv(:,2);

w = sensor_data;



[u_, v_] = global_coord_to_uv(slam_txy(:,1), slam_txy(:,2), slam_txy(:,3), slam_h);


au_1 = diff(u_)./diff(slam_txy(:,1));
au_1 = [au_1(1);au_1];
au_1 = lowpass(au_1,pass_band,mocap_Fs);

av_1 = diff(v_)./diff(slam_txy(:,1));
av_1 = [av_1(1);av_1];
av_1 = lowpass(av_1,pass_band,mocap_Fs);

diff_slam_h = diff(slam_h);
need_adjustment_idx = abs(diff_slam_h)>5;
diff_slam_h(need_adjustment_idx) =  diff_slam_h(need_adjustment_idx) - 2*pi*sign(diff_slam_h(need_adjustment_idx));
slam_r   = diff_slam_h./diff_slam_states(:,1);
slam_r   = [slam_r(1); slam_r];
slam_r_original = slam_r;
slam_r   = lowpass(slam_r,pass_band*4,mocap_Fs);


diff_mocap_states = diff(mocap_txy);
diff_mocap_h = diff(mocap_h);
need_adjustment_idx = abs(diff_mocap_h)>5;
diff_mocap_h(need_adjustment_idx) =  diff_mocap_h(need_adjustment_idx) - 2*pi*sign(diff_mocap_h(need_adjustment_idx));
mocap_r   = diff_mocap_h'./diff_mocap_states(:,1);
mocap_r   = [mocap_r(1); mocap_r];
mocap_r_original = mocap_r;
mocap_r   = lowpass(mocap_r,pass_band*4,mocap_Fs);




ar_1 = diff(slam_r)./diff(slam_txy(:,1));
ar_1 = [ar_1(1);ar_1];
ar_1 = lowpass(ar_1,pass_band,mocap_Fs);


% clf(gcf)
% plot(state_data(1:end,1),state_data(1:end,6))
% hold on
% plot(slam_txy(:,1),mocap_uv(:,2))
% disp(i)
% pause(0.05)
% end
% obtain r, au av ar
%obtain au, av, ar
% imu_data(:,4) = lowpass(imu_data(:,4),pass_band_imu,imu_Fs);
imu_data(:,4) = [zeros(12,1); imu_data(1:end-12,4)];
imu_data(:,3) = [zeros(12,1); imu_data(1:end-12,3)];
imu_data(:,2) = [zeros(12,1); imu_data(1:end-12,2)];


imu_r = lowpass(imu_data(:,4),pass_band_imu*4,imu_Fs);
imu_ar =  diff(imu_data(:,4))./diff(imu_data(:,1));
imu_ar = [imu_ar(1,:);imu_ar];
imu_ar = lowpass(imu_ar,pass_band_acc*0.1,imu_Fs);
imu_data(:,2:3)= lowpass(imu_data(:,2:3),pass_band_imu,imu_Fs);

imu_v = match_trajectories(imu_data(:,1)', slam_txy(:,1)',slam_uv(:,2)')';
imu_au = imu_data(:,2)+imu_v.*imu_data(:,4)+0.2329;
imu_au(isnan(imu_au)) = 0;
imu_au = lowpass(imu_au,pass_band_acc*0.1,imu_Fs);
imu_u = match_trajectories(imu_data(:,1)', slam_txy(:,1)',slam_uv(:,1)')';
imu_av = imu_data(:,3)-imu_u.*imu_data(:,4)+0.04;
imu_av(isnan(imu_av)) = 0;
imu_av = lowpass(imu_av,pass_band_acc*0.1,imu_Fs);



% elim gaps




imu_t_new = imu_data(:,1);
motor_t_new = motor_cmd(:,1);
delta_t_new = delta_cmd(:,1);
w_t_new = w(:,1);
auto_flag_t_new = auto_flag_data(:,1);
slam_t_new = slam_txy(:,1);


delta_t_new = delta_cmd(:,1);


% match data
t_ = slam_t_new;


[auto_flag_,...
    au_,...
    av_,...
    ar_,...
    r_,...
    motor_cmd_,...
    delta_cmd_,...
    w_,...
    maneuv_list,...
    u_est,...
    v_est,...
    r_est,...
    w_est,...
    u_mocap_,...
    v_mocap_,...
    r_mocap_,...
    h_mocap_,...
    mocap_x_,...
    mocap_y_] = match_trajectories(t_...
                        ,auto_flag_t_new,auto_flag_data(:,2)... 
                        ,imu_t_new,imu_au...
                        ,imu_t_new,imu_av...
                        ,imu_t_new,imu_ar...
                        ,imu_t_new,imu_r...
                        ,motor_t_new,motor_cmd(:,2)...
                        ,delta_t_new,delta_cmd(:,2)...
                        ,w_t_new,w(:,2)...
                        ,auto_flag_t_new,maneuver_type_data...
                        ,estimator_data(:,1), estimator_data(:,2)...
                        ,estimator_data(:,1), estimator_data(:,3)...
                        ,estimator_data(:,1), estimator_data(:,4)...
                        ,estimator_data(:,1), estimator_data(:,5)...
                        ,mocap_data(:,1), mocap_u...
                        ,mocap_data(:,1), mocap_v...
                        ,mocap_data(:,1), mocap_r...
                        ,mocap_data(:,1), mocap_h...
                        ,mocap_data(:,1), mocap_x...
                        ,mocap_data(:,1), mocap_y);
 

%% filtering signals

auto_flag_ = logical(auto_flag_);

%filtering out auto driving portion:
t_auto = t_(auto_flag_);
x_auto = x_(auto_flag_);
y_auto = y_(auto_flag_);
slam_h_auto = slam_h(auto_flag_);
au_auto = au_1(auto_flag_);
av_auto = av_1(auto_flag_);
ar_auto = ar_(auto_flag_)';
manuev_list_auto = maneuv_list(auto_flag_);
slam_txy_auto = slam_txy(auto_flag_, :);
u_auto = u_(auto_flag_);
v_auto = v_(auto_flag_);
r_auto = r_(auto_flag_)';
slam_r_auto = slam_r(auto_flag_);
w_auto = w_(auto_flag_);
motor_cmd_auto = motor_cmd_(auto_flag_)';
delta_cmd_auto = delta_cmd_(auto_flag_)';
u_est_auto = u_est(auto_flag_);
v_est_auto = v_est(auto_flag_);
r_est_auto = r_est(auto_flag_)';
% slam_h_auto = slam_h_(auto_flag_);

ud_auto = ud(auto_flag_);
rd_auto = rd(auto_flag_)';

u_mocap_auto = u_mocap_(auto_flag_);
v_mocap_auto = v_mocap_(auto_flag_);
r_mocap_auto = r_mocap_(auto_flag_);
h_mocap_auto = h_mocap_(auto_flag_);
x_mocap_auto = mocap_x_(auto_flag_);
y_mocap_auto = mocap_y_(auto_flag_);
h_slam_auto = slam_h(auto_flag_);






%Obtain indices for straight and curved driving
% 
r_auto = slam_r_auto;



vf = v_auto + lf*r_auto;
vr = v_auto - lr*r_auto;


uvf = [u_auto'; vf'];
uvr = [u_auto'; vr']';

uv_wf = reshape((rotmat(delta_cmd_auto)*uvf(:)),2,[])';
% 
% 
beta_f = (w_auto' - u_auto) ./ max([sqrt(u_auto.^2 +0.05), sqrt(w_auto'.^2 +0.05)],[],2);
beta_r = (w_auto' - u_auto) ./ max([sqrt(u_auto.^2 +0.05), sqrt(w_auto'.^2 +0.05)],[],2);


% beta_f = (w_' + u_) ./ max([sqrt(u_.^2 +0.2), sqrt(w_'.^2 +0.2)],[],2);
% beta_r = (w_' + u_) ./ max([sqrt(u_.^2 +0.05), sqrt(w_'.^2 +0.05)],[],2);


%Fywf = (lr*m*(av_'+u_.*r_')+r_'*Izz-(lf+lr)*sin(delta).*F_list')./((lf+lr)*cos(delta)');
% Fywr = (m*(av_auto'+u_auto.*r_auto')-(r_auto'*Izz/lf))/(1+lr/lf);
Fxwf = m*au_auto*0.4;
Fxwr = m*au_auto*0.6;

% dw = 1*(motor_cmd_auto - w_auto);
% 
% Fxwf = m*dw'*0.4;
% Fxwr = m*dw'*0.6;

alpha_f = atan(uv_wf(:,2) ./ sqrt((uv_wf(:,1).^2)+0.05));
alpha_r = atan(uvr(:,2) ./ sqrt((uvr(:,1).^2)+0.05));

% % 
Fywf = (lr*m*(av_auto+u_auto.*r_auto)+ar_auto*Izz-(lf+lr)*sin(delta_cmd_auto).*Fxwf)./((lf+lr)*cos(delta_cmd_auto));
Fywr = (m*(av_auto+u_auto.*r_auto)-(ar_auto*Izz/lf))/(1+lr/lf);


% Fywf = (lr*m*(av_'+u_.*r_')+r_'*Izz-(lf+lr)*sin(delta_cmd_').*Fxwf)./((lf+lr)*cos(delta_cmd_'));
% Fywr = (m*(av_'+u_.*r_')-(r_'*Izz/lf))/(1+lr/lf);

% u_ = [nan(15,1);u_(1:end-15)];
% v_ = [nan(15,1);v_(1:end-15)];

%%
plot_animation =  1;

auto_filter_start = find(diff(auto_flag_)==1) + 1;
auto_filter_end = find(diff(auto_flag_)==-1) + 1;
auto_filter_start_2 = find(diff(auto_flag_data(:,2))==1) + 1;
maneuver_list = maneuver_type_data(auto_filter_start_2) ;
force_data = [Fxwf Fxwr Fywf Fywr];
slip_data = [beta_f beta_r alpha_f alpha_r];
% 
% maneuver_list(17) = [];
% maneuver_list(33) = [];
% auto_filter_start(17) = [];
% auto_filter_start(33) = [];



 
%model_consts = [ Cw,       delta_amp,      Caf1,   Caf2,   %deltaoff,       Car1,  Car2,      I,       lf,  mass_ratio]
model_consts =  [3.41,          -1.0,        60,      0.5,      -0.012,         100,     0.5,    0.11,     0.20,         0.4];

J = model_fit_cost_function2(model_consts...
                           ,i_ini...
                           ,N...
                           ,T...
                           ,slam_txy...
                           ,slam_h...
                           ,slam_spd_total...
                           ,slam_spddir...
                           ,[u_, v_]...
                           ,slam_r...
                           ,model_type...
                           ,delta_cmd_...
                           ,motor_cmd_...
                           ,w_...
                           ,consts...
                           ,slip_data...
                           ,force_data...
                           ,plot_animation...
                           ,plot_figure_1 ...
                           ,auto_filter_start...
                           ,auto_flag_...
                           ,maneuver_list...
                           ,ud...
                           ,vd...
                           ,rd...
                           ,u_est'...
                           ,v_est'...
                           ,r_est')