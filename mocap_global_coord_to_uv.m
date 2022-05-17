function [u, v] = mocap_global_coord_to_uv(t, x, y, quat)
%{

    !!!USE FOR MOCAP DATA ONLY!!!

    function purpose: convert global xy coodinate of the vehicle in to its
    longitudinal and lateral velocity.

   N = the number of data points.

   INPUTS: 
    -  t: N by 1 time vector
    -  x: N by 1 x position vector
    -  y: N by 1 y position vector
    -  h: N by 1 heading vector

    OUTPUTS: 
    -  u: N by 1 longitudinal velocity vector
    -  v: N by 1 lateral velocity vector

%}

mocap_h = [];

for i = 1:length(t)

    rot1 = [1  0  0;
            0  0 -1;
            0  1  0]; %Initial car rotation
    
    roth = rot1'*quat2rotm(quat)*rot1; 
    h_temp = rotm2eul(roth);
    
    mocap_h = [mocap_h h_temp(1)];
end



mocap_txy = [t x y];


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

    
end