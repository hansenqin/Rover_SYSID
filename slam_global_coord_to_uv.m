function [u,v] = slam_global_coord_to_uv(t, x, y, h)
%{

    !!!USE FOR SLAM DATA ONLY!!!

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

    RR=rotmat(-h);
    diff_slam_states = diff([t x y]);
    slam_spd_vec = diff_slam_states(:,2:3) ./  diff_slam_states(:,1);
    slam_spd_vec = [slam_spd_vec(1,:);slam_spd_vec];

    slam_spd_one_col =  slam_spd_vec';
    slam_spd_one_col =  slam_spd_one_col(:);

    slam_uv = RR * slam_spd_one_col;
    slam_uv = reshape(slam_uv,2,[])';
    slam_uv = lowpass(slam_uv,pass_band ,mocap_Fs);
    u = slam_uv(:,1);
    v = slam_uv(:,2);
    
end