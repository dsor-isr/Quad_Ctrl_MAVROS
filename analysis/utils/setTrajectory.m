function trajectory = setTrajectory(parent,vehicles,trajectory)
% Loads trajectories from .csv files and creates timeseries from that data.
% 
% This function searches for the file <trajectory_name>.csv located at 
% /parent/trajectories/. In other words, somewhere in the file system there
% must exist a folder named 'trajectories' where the csv trajectory file
% can be found.
% 
% Each line of the trajectory file must contain 15 entries, in a
% particular order: 
% 01 - time instant [s], 
% 02 - position in x [m], 
% 03 - position in y [m],
% 04 - position in z [m], 
% 05 - linear velocity in x [m/s], 
% 06 - linear velocity in y [m/s], 
% 07 - linear velocity in z [m/s],
% 08 - linear acceleration in x [m/s^2],
% 09 - linear acceleration in y [m/s^2], 
% 10 - linear acceleration in z [m/s^2], 
% 11 - linear jerk in x [m/s^3], 
% 12 - linear jerk in y [m/s^3], 
% 13 - linear jerk in z [m/s^3],
% 14 - yaw angle [rad],
% 15 - first time derivative of the yaw angle [rad/s]
% 
% The lines in the file must be sorted according to the time instants 
% (first entry), in descending order.
% 
% As an example, below, there is a snippet of a trajectory which is a 
% step function in the z-axis from z = 1.5 m to z = 2.0 m, with the
% duration of 5 seconds.
% 
% % % % % % % % % % % % % % % % % 
% 5,0,0,2,0,0,0,0,0,0,0,0,0,0,0
% 4.99,0,0,2,0,0,0,0,0,0,0,0,0,0,0
% 4.98,0,0,2,0,0,0,0,0,0,0,0,0,0,0
% (...)
% 0.02,0,0,1.5,0,0,0,0,0,0,0,0,0,0,0
% 0.01,0,0,1.5,0,0,0,0,0,0,0,0,0,0,0
% 0,0,0,1.5,0,0,0,0,0,0,0,0,0,0,0
% % % % % % % % % % % % % % % % %
% 
% Input:
% - parent: char array specifying a path from the root to folder
%           'trajectories';
% 
% - vehicles: Array of 'vehicle' structs (use the round brackets operator 
%             to access the elements of the array). When using this
%             function each 'vehicle' must have the field 'mass' specified;
% 
% - trajectory: Array of 'trajectory' structs. Each struct of the array
%               must have the field 'name' specified. This field
%               corresponds to the name of the csv file to be opened.
% 
% Output:
% - trajectory: Array of 'trajectory' structs with the information loaded
%               from the csv file organised into multiple 'timeseries' 
%               objects.
% 
% JOAO PINTO (2021-09-21)

    N = length(trajectory);
    
    for j = 1:N
        
        location = strcat(parent,'trajectories/',trajectory(j).name,'.csv');
        Data = flip( csvread(location) );

        trajectory(j).pose = timeseries();
        trajectory(j).position = timeseries();
        trajectory(j).velocity = timeseries();
        trajectory(j).rotm = timeseries();
        trajectory(j).euler = timeseries();
        trajectory(j).rates = timeseries();
        trajectory(j).thrust = timeseries();
        trajectory(j).throttle = timeseries();

        trajectory(j).position.Time = Data(:,1);
        trajectory(j).position.Data = Data(:,2:4);

        trajectory(j).velocity.Time = Data(:,1);
        trajectory(j).velocity.Data = Data(:,5:7);

        R = acc2R(Data(:,8:10),Data(:,14));

        trajectory(j).rotm.Time = Data(:,1);
        trajectory(j).rotm.Data = reshapeRotm( R );

        trajectory(j).euler.Time = Data(:,1);
        trajectory(j).euler.Data = zxy_R2Euler( R );

        trajectory(j).thrust.Time = Data(:,1);
        trajectory(j).thrust.Data = acc2Thr( vehicles(j).mass, Data(:,8:10) );

        trajectory(j).rates.Time = Data(:,1);
        trajectory(j).rates.Data = getRates( vehicles(j).mass, ...
                                        Data(:,11:13), Data(:,15), R, ...
                                                trajectory(j).thrust.Data );

        trajectory(j).throttle.Time = Data(:,1);

    end
end

