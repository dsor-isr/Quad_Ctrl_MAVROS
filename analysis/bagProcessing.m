
function [uav,trajectory] = bagProcessing(ts,trajectory,pose,vel,cmd,vehicle)
% Process data collected from a rosbag in order to make plots and animations.
% JOAO PINTO (2021-09-21)

    len = length(pose.Data);
    len_cmd = length(cmd.Data);

    Q.pose = NaN(len,4);
    Q.pose(1:len,:) = pose.Data(:,4:end);
    Q.pose = Q.pose(:,[4 1 2 3]);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Attitude (Quaternions) --> Rotation Matrices
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Note that MATLAB and ROS order the quaternion components differently.
    % MATLAB puts the real component in the first position, while ROS, in the
    % last
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    Q.cmd = NaN(len_cmd,4);
    Q.cmd(1:len_cmd,:) = cmd.Data(:,2:end);
    Q.cmd = Q.cmd(:,[4 1 2 3]);

    R = quaternion2R(Q.pose);
    R_cmd = quaternion2R(Q.cmd);

    R1 = reshapeRotm(R);
    R1_cmd = reshapeRotm(R_cmd);

    pose.Data(:,4:12) = R1;
    cmd.Data(:,2:10) = R1_cmd;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Shift Indices (sh)
    % Find the time interval in which the vehicle is in OFFBOARD mode
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % START (OFFBOAD)
    uav.cmd.sh(1).idx = find( ...
            abs( cmd.Data(3:end,1) - cmd.Data(1:end-2,1) ) > 1e-1, 1);           
    
    uav.cmd.sh(1).idx = uav.cmd.sh(1).idx + 2;
    uav.cmd.sh(1).val = round( cmd.Time( uav.cmd.sh(1).idx ), 2 );

    % FINISH (OFFBOAD)
    uav.cmd.sh(2).idx = length( cmd.Time );
    uav.cmd.sh(2).val = round( cmd.Time(end), 2 );

    %%%%%%%%%%%%%%%%%%%

    % TAKE-OFF
    uav.vel.sh(1).idx = 1;
    uav.vel.sh(1).val = round( vel.Time( uav.vel.sh(1).idx ), 2 );

    % START (OFFBOARD)
    uav.vel.sh(2).idx = find( abs( vel.Time - uav.cmd.sh(1).val ) <= 3*ts );             
    uav.vel.sh(2).idx = ...
                uav.vel.sh(2).idx( round( length( uav.vel.sh(2).idx )/2 ) );
    uav.vel.sh(2).val = round( vel.Time( uav.vel.sh(2).idx ), 2 );
    
    % FINISH (OFFBOAD)
    uav.vel.sh(3).idx = find( abs( vel.Time - uav.cmd.sh(2).val ) <= 3*ts );           
    uav.vel.sh(3).idx = ...
                uav.vel.sh(3).idx( round( length( uav.vel.sh(3).idx )/2 ) );
    uav.vel.sh(3).val = round( vel.Time( uav.vel.sh(3).idx ), 2 );

    % LANDING
    uav.vel.sh(4).idx = length( vel.Time );
    uav.vel.sh(4).val = round( vel.Time(end), 2 );

    %%%%%%%%%%%%%%%%%%%
    
    % TAKE-OFF
    uav.pose.sh(1).idx = 1;
    uav.pose.sh(1).val = round( pose.Time( uav.pose.sh(1).idx ), 2 );

    % START (OFFBOARD)
    uav.pose.sh(2).idx = find( ...
                            abs( pose.Time - uav.cmd.sh(1).val ) <= 3*ts);
    uav.pose.sh(2).idx = ...
            uav.pose.sh(2).idx( round( length( uav.pose.sh(2).idx )/2 ) );
    uav.pose.sh(2).val = round( pose.Time( uav.pose.sh(2).idx ), 2 );
    
    % FINISH (OFFBOAD)
    uav.pose.sh(3).idx = find( ...
                            abs( pose.Time - uav.cmd.sh(2).val ) <= 3*ts );
    uav.pose.sh(3).idx = ...
            uav.pose.sh(3).idx( round( length( uav.pose.sh(2).idx )/2 ) );
    uav.pose.sh(3).val = round( pose.Time( uav.pose.sh(3).idx ), 2 );

    % LANDING
    uav.pose.sh(4).idx = length( pose.Time );
    uav.pose.sh(4).val = round( pose.Time(end), 2 );

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Find when the vehicle takes off
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    sh_prev = find( ...
        sum( ( pose.Data(2:end,1:3) - pose.Data(1:end-1,1:3) ).^2, 2 ) > 1e-4, 1);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    sh_prev = round(pose.Time(sh_prev),2);
    sh_prev_a = round( sh_prev/ts ) + 1;
    sh_a = round( uav.pose.sh(2).val/ts ) + 1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    trajectory.pose = ...
        timeseries( [trajectory.position.Data trajectory.rotm.Data], ...
                                                trajectory.position.Time);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    trajectory_pos = [zeros(sh_a,4); ...
                        trajectory.position.Time trajectory.position.Data];

    trajectory_pos(1:sh_a,1) = 0:ts:uav.pose.sh(2).val;
    
    trajectory_pos(sh_a+1:end,1) = ...
                    trajectory_pos(sh_a+1:end,1) + uav.pose.sh(2).val + ts;

    trajectory_pos(1:sh_prev_a-1,2:end) = ...
                            repmat( pose.Data(1,1:3), sh_prev_a - 1, 1 );

    trajectory_pos(sh_prev_a:sh_a,2:end) = ...
        repmat( trajectory.position.Data(1,1:3), sh_a - sh_prev_a + 1, 1 );
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    trajectory.position = ...
                timeseries( trajectory_pos(:,2:4), trajectory_pos(:,1) );
            
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    uav.pose.position = timeseries( pose.Data(:,1:3), pose.Time );
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    uav.pose.rotm = timeseries( pose.Data(:,4:end), pose.Time );
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    uav.pose.euler = timeseries( ...
            zxy_R2Euler( reshape(pose.Data(:,4:end)',3,3,[]) )*180/pi,...
                                                                pose.Time);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    trajectory.velocity.Time = uav.vel.sh(2).val + trajectory.velocity.Time;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    trajectory.rates.Time = uav.vel.sh(2).val + trajectory.rates.Time;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    trajectory.euler.Time = uav.cmd.sh(1).val + trajectory.euler.Time;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    trajectory.euler.Data = trajectory.euler.Data*180/pi;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    trajectory.throttle.Data = ...
        vehicle.thr_curve(trajectory.thrust.Data,trajectory.velocity.Data);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    trajectory.throttle.Time = uav.cmd.sh(1).val + trajectory.throttle.Time;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    uav.vel.lin = timeseries( vel.Data(:,1:3), vel.Time );
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    uav.vel.ang = timeseries( vel.Data(:,4:6), vel.Time );
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    uav.cmd.thr = timeseries( cmd.Data( uav.cmd.sh(1).idx:end,1 ), ...
                                    cmd.Time( uav.cmd.sh(1).idx:end ) );
                                
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % CHOOSE EITHER ORIENTATION OR ANGULAR RATE COMMANDS!
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % ORIENTATION COMMANDS
    uav.cmd.euler = timeseries( ...
        zxy_R2Euler( reshape(cmd.Data( uav.cmd.sh(1).idx:end,2:end )', ...
                    3,3,[]) )*180/pi, cmd.Time( uav.cmd.sh(1).idx:end ) );
    
    % ANGULAR RATE COMMANDS
%     uav.cmd.rates = timeseries( cmd.Data( uav.cmd.sh(1).idx:end,2:end ), ...
%                                         cmd.Time( uav.cmd.sh(1).idx:end ) );
                
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    uav.cad = vehicle.cad;
    uav.id = vehicle.id;
    uav.type = vehicle.type;
    
end

