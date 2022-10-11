%% Plot Mocap X vs Wheel Encoder Velocity
close all
hold on
plot(sysID.wheel_encoder.time, sysID.rw*sysID.wheel_encoder.encoder_velocity, 'b', 'DisplayName','Mocap U');
plot(sysID.vehicle_states_from_mocap.time, sysID.vehicle_states_from_mocap.x, 'r', 'DisplayName','Wheel Encoder U');
legend;

%% Plotting raw Mocap X vs Wheel Encoder Displacement
close all
hold on
plot(sysID.wheel_encoder.time, -(sysID.wheel_encoder.encoder_position-sysID.wheel_encoder.encoder_position(1))*2*pi*sysID.rw/(4096))
plot(sysID.vehicle_states_from_mocap.time, sysID.vehicle_states_from_mocap.x - sysID.vehicle_states_from_mocap.x(1), 'r')

%% Plotting u vs u form encoders
close all;
hold on;
idx = 5;
plot(sysID.trajectories(idx).time, sysID.trajectories(idx).u, 'r', 'DisplayName','Mocap Longitudnal Velocity (m/s)');
plot(sysID.trajectories(idx).time, sysID.trajectories(idx).encoder_velocity*sysID.rw, 'b', 'DisplayName','Wheel Longitudnal Velocity (m/s)');
plot(sysID.trajectories(idx).time, sysID.trajectories(idx).udot, 'k', 'DisplayName','Mocap Longitudnal Acceleration (m/s^2)');
title("Trajectory " + idx)
xlabel("Time");
legend;

%% Plotting slam u, wheel u, lambda, udot
scatter(sysID.trajectorty.time, sysID.trajectorty.u, 'r', 'DisplayName','SLAM U');
plot(sysID.vehicle_encoder.time, sysID.vehicle_encoder.encoder_velocity, 'b', 'DisplayName','Wheel U');
plot(sysID.vehicle_states_from_slam.time, sysID.vehicle_states_from_slam.udot, 'r', 'DisplayName','SLAM UDot');
plot(sysID.vehicle_states_from_slam.time, lambda, 'r', 'DisplayName','SLAM U');

%% Plotting Raw SLAM X, Y, H
scatter(sysID.vehicle_states_from_slam.time, sysID.vehicle_states_from_slam.x, 'r', 'DisplayName','Raw SLAM X');
scatter(sysID.vehicle_states_from_slam.time, sysID.vehicle_states_from_slam.y, 'g', 'DisplayName','Raw SLAM Y');
scatter(sysID.vehicle_states_from_slam.time, sysID.vehicle_states_from_slam.h, 'b', 'DisplayName','Raw SLAM H');

%% Ploting trajectories
hold on
for i = 1:length(sysID.trajectories)
    scatter(sysID.trajectories(i).time, sysID.trajectories(i).x)
end

%% Plotting trajectory u, encoder_velocity, udot
close all;
hold on;
for i = 1:length(sysID.trajectories) 
    idx = i;
    plot(sysID.trajectories(idx).time, sysID.trajectories(idx).u, 'r', 'DisplayName','SLAM U');
    plot(sysID.trajectories(idx).time, sysID.trajectories(idx).encoder_velocity*sysID.rw, 'b', 'DisplayName','Wheel U');
    plot(sysID.trajectories(idx).time, sysID.trajectories(idx).udot, 'k', 'DisplayName','SLAM U Dot');
end

