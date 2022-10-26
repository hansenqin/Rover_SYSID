clear all;
close all;
clc;
map = load('mocap_lateral_10_09_2.mat');
imu = cell2mat(map.messageStruct_map('/imu/data'));
mocap = cell2mat(map.messageStruct_map('/mocap'));

% joint_states = cell2mat(map.messageStruct_map('/joint_states'));
% state = cell2mat(map.messageStruc_map('/state_estimator/states'));
rover_debug_state_out = cell2mat(map.messageStruct_map('/state_out/rover_debug_state_out'));
tf = cell2mat(map.messageStruct_map('/tf'));
% current = cell2mat(map.messageStruct_map('/vesc/commands/motor/current'));
unsmoothed_speed = cell2mat(map.messageStruct_map('/vesc/commands/motor/unsmoothed_speed'));
odom = cell2mat(map.messageStruct_map('/vesc/odom'));
vesc_sensors_core = cell2mat(map.messageStruct_map('/vesc/sensors/core'));
servo_position_command = cell2mat(map.messageStruct_map('/vesc/sensors/servo_position_command'));

