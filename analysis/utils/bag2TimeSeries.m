function [pose,vel,cmd] = bag2TimeSeries(ns,bag)
% From a 'BagSelection' object returns timeseries with messages from topics
% 
% In order to use this function, it is necessary to extend the set of 
% message types which are supported in ROS Toolbox. To do so, one needs to
% create custom messages from the MAVROS package in MATLAB.
% 
% Follow the instructions in this link from the MATLAB Answers to readily
% create the necessary custom messages:
% https://www.mathworks.com/matlabcentral/answers/355617-robot-system-toolbox-doesn-t-support-the-message-type-mavros_msgs-positiontarget#answer_280834
% 
% Inputs:
% - ns: char array specifying the namespace of the vehicle;
% - bag: 'robotics.ros.BagSelection' object obtained from using the
%        function ROSBAG (run script unpackBag to obtain this object).
% 
% Outputs:
% - pose, vel, cmd: 'timeseries' objects containing messages regarding the
%                   pose, linear and angular velocities, and control inputs, 
%                   respectively, from the vehicle associated with the 
%                   namespace 'ns'.
%
% JOAO PINTO (2021-09-21)

    pose_bag = select(bag,'Topic',strcat(ns,'/mavros/local_position/pose'));
    vel_bag = select(bag,'Topic', ...
                        strcat(ns,'/mavros/local_position/velocity_local'));
    cmd_bag = select(bag,'Topic',strcat(ns,'/mavros/setpoint_raw/attitude'));

    pose = timeseries(pose_bag, ...
                            'Pose.Position.X',...
                                    'Pose.Position.Y',...
                                        'Pose.Position.Z',...
                                            'Pose.Orientation.X',...
                                                'Pose.Orientation.Y',...
                                                    'Pose.Orientation.Z',...
                                                            'Pose.Orientation.W');

    vel = timeseries(vel_bag, 'Twist.Linear.X', 'Twist.Linear.Y', ...
                                    'Twist.Linear.Z', 'Twist.Angular.X', ... 
                                            'Twist.Angular.Y', 'Twist.Angular.Z');
                                        
    %%%%%%%%%%%%%%%%%%%%%%%                                  
    % Thrust + Ang. rates  
    %%%%%%%%%%%%%%%%%%%%%%%
    % cmd = timeseries(cmd_bag, ...
    %                     'Thrust', 'BodyRate.X', 'BodyRate.Y', 'BodyRate.Z');

    %%%%%%%%%%%%%%%%%%%%%%%                                  
    % Thrust + Orientation
    %%%%%%%%%%%%%%%%%%%%%%%
    cmd = timeseries(cmd_bag, ...
                        'Thrust', 'Orientation.X', 'Orientation.Y', ...
                                            'Orientation.Z', 'Orientation.W');
                                        
    %%%%%%%%%%%%%%%%%%%%%%%                                  
    % Position Setpoints
    %%%%%%%%%%%%%%%%%%%%%%%
    % ref_pose_bag = select(bag,'Topic',...
    %                         strcat(ns,'/mavros/setpoint_position/local'));
    % 
    % ref_pose = timeseries(ref_pose_bag, 'Pose.Position.X',...
    %                                             'Pose.Position.Y',...
    %                                                     'Pose.Position.Z');
    
    pose.Time = pose.Time - bag.StartTime;
    vel.Time = vel.Time - bag.StartTime;
    cmd.Time = cmd.Time - bag.StartTime;

end

