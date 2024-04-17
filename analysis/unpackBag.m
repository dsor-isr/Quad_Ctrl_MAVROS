% Make plots and animations from data collected in a rosbag and evaluate tracking performance.
% JOAO PINTO (2021-09-21)

clear
close all

addpath('utils')

%%%%%%%%%%%%%%%%%%%%%%%%
% Sampling time
ts = 0.01;
%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%
% CHOOSE EITHER 'iris' or 'intel'
% EDIT setVehicles.m if necessary
%%%%%%%%%%%%%%%%%%%%%%%%

vehicles(1).type = 'iris';
vehicles(2).type = 'iris';

% vehicles(1).type = 'intel';
% vehicles(2).type = 'intel';

vehicles = setVehicles(vehicles);

%%%%%%%%%%%%%%%%%%%%%%%%
% Select bag and trajectories
%%%%%%%%%%%%%%%%%%%%%%%%

% Choose your parent directory!
parent = pwd;
parent = parent(1:end-8);

% Default values multi_quad_ctrl_sitl.launch
% and multi_quad_ctrl.launch
bag.name = 'gazebo_test';
trajectory(1).name = 'steps/z_steps/z_step_1';
trajectory(2).name = 'steps/z_steps/z_step_2';

%%%%%%%%%%%%%%%%%%%%%%%%
N = length(trajectory);
%%%%%%%%%%%%%%%%%%%%%%%%

bag.location = strcat(parent,'bags/',bag.name,'.bag');
bag.Data = rosbag(bag.location);

trajectory = setTrajectory(parent,vehicles,trajectory);

%%%%%%%%%%%%%%%%%%%%%%%%
% Pre-allocate structs and timeseries
%%%%%%%%%%%%%%%%%%%%%%%%

ns = cell(1,N);

pose = repmat(timeseries(),1,N);
vel = repmat(timeseries(),1,N);
cmd = repmat(timeseries(),1,N);

%%%%%%%%%%%%%%%%%%%%%%%%
% 'x' is a template of struct 'uav'
%%%%%%%%%%%%%%%%%%%%%%%%

x.id = [];
x.type = '';
x.cad = struct('V',[],'F',[],'facecolors',[]);
x.pose = struct('position',timeseries(), ...
                    'euler',timeseries(),'rotm',timeseries(),'sh',[]);
x.vel = struct('lin',timeseries(),'ang',timeseries(),'sh',[]);

%%%%%%%%%%%%%%%%%%%%%%%%
% CHECK bagProcessing.m
% CHECK bag2TimeSeries.m
%%%%%%%%%%%%%%%%%%%%%%%%

% THRUST + ORIENTATION COMMANDS
x.cmd = struct('thr',timeseries(),'euler',timeseries(),'sh',[]);

% THRUST + ANGULAR RATE COMMANDS
% x.cmd = struct('thr',timeseries(),'rates',timeseries(),'sh',[]);

%%%%%%%%%%%%%%%%%%%%%%%%
% Create array of 'uav' structs
%%%%%%%%%%%%%%%%%%%%%%%%

uav = repmat(x,1,N);

%%%%%%%%%%%%%%%%%%%%%%%%

for j = 1:N
    ns{j} = strcat('/uav/uav',num2str(j-1));
%     ns{j} = '/uav/uav1';

    [pose(j),vel(j),cmd(j)] = bag2TimeSeries(ns{j},bag.Data);
    
    [uav(j),trajectory(j)] = bagProcessing(ts,trajectory(j),pose(j),...
                                                vel(j),cmd(j),vehicles(j));
end

%%%%%%%%%%%%%%%%%%%%%%%%
% Plots and Animations
%%%%%%%%%%%%%%%%%%%%%%%%

bagPlots(ts,uav,trajectory)

t_start = trajectory(1).pose.Time(1);
t_finish = trajectory(1).pose.Time(end);

S = 2;
vt = t_start:(t_finish-t_start)/(S-1):t_finish;
play_speed = 125;

trajectoryAnim(ts,play_speed,t_start,t_finish,uav,trajectory)
trajectoryPlot3D(ts,vt,t_start,t_finish,uav,trajectory)


