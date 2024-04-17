function bagPlots(ts,uav,trajectory)
% Draws plots of the data collected from the selected rosbag.
% 
% Makes plots of position, orientation, linear and angular velocities, and
% throttle data gathered during flight together with the reference 
% trajectories. 
% 
% The different flight phases are depicted using distinct colours. 
% The colour PURPLE is linked to taking-off, the colour GREEN, to when the
% vehicle is receiving commands from an offboard computer, and the colour 
% BLUE, to landing. Inputs, trajectories and observed data are 
% distinguished from one another by using different shades of the same 
% colour (check section labelled COLOURS).
% 
% Inputs:
% - ts: Sampling time of the controller, or equivalently, the inverse of
%       the working frequency (in seconds);
% 
% - uav: Array of 'uav' structs (run script unpackBag to obtain this array).
%        Note that one can select which UAVs will be drawn in the
%        animation. One may choose to draw all UAVs, or simply a subset of
%        them. Use the round brackets operator to access the elements of 
%        the array;
% 
% - trajectory: Array of 'trajectory' structs (run script unpackBag to
%               obtain this array). Note that trajectory(k) is tracked by
%               uav(k). One may choose to draw all trajectories, or simply 
%               a subset of them. Use the round brackets operator to access 
%               the elements of the array;
% 
% JOAO PINTO (2021-09-21)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % COLOURS
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%
    % TAKE-OFF
    %%%%%%%%%%%%%%%
    
    purple1 = '#7F00FF'; % REFERENCE TRAJECTORY
    purple1 = sscanf(purple1(2:end),'%2x%2x%2x',[1 3])/255;

    purple2 = '#CC99FF'; % OBSERVED DATA
    purple2 = sscanf(purple2(2:end),'%2x%2x%2x',[1 3])/255;
    
    %%%%%%%%%%%%%%%
    % OFFBOARD
    %%%%%%%%%%%%%%%

    green1 = '#528016'; % REFERENCE TRAJECTORY
    green1 = sscanf(green1(2:end),'%2x%2x%2x',[1 3])/255;

    green2 = '#8BD925'; % OBSERVED DATA
    green2 = sscanf(green2(2:end),'%2x%2x%2x',[1 3])/255;
    
    green3 = '#C2FFB0'; % INPUT COMMANDS
    green3 = sscanf(green3(2:end),'%2x%2x%2x',[1 3])/255;
    
    %%%%%%%%%%%%%%%
    % LANDING
    %%%%%%%%%%%%%%%
    
    blue1 = '#1FE9FF'; % OBSERVED DATA
    blue1 = sscanf(blue1(2:end),'%2x%2x%2x',[1 3])/255;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    L = length(uav);
    
    for n = 1:L
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Intervals
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        T1.pose = uav(n).pose.sh(1).idx:uav(n).pose.sh(2).idx;
        T1.vel = uav(n).vel.sh(1).idx:uav(n).vel.sh(2).idx;

        T2.pose = uav(n).pose.sh(2).idx:uav(n).pose.sh(3).idx;
        T2.vel = uav(n).vel.sh(2).idx:uav(n).vel.sh(3).idx;

        T3.pose = uav(n).pose.sh(3).idx:uav(n).pose.sh(4).idx;
        T3.vel = uav(n).vel.sh(3).idx:uav(n).vel.sh(4).idx;

        T1.ref.pose = ...
            round( uav(n).pose.sh(1).val/ts+1:uav(n).pose.sh(2).val/ts+1 );
        
        T2.ref.pose = ...
            round( uav(n).pose.sh(2).val/ts+1:uav(n).pose.sh(3).val/ts+1 );

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % POSITION
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        figure
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        subplot(3,1,1);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        plot(uav(n).pose.position.Time(T1.pose), ...
                uav(n).pose.position.Data(T1.pose,1), ...
                        'Color',purple2,'Linewidth',1.5)
        hold on
        plot(trajectory(n).position.Time(T1.ref.pose), ...
                trajectory(n).position.Data(T1.ref.pose,1), ...
                            'Color',purple1,'Linewidth',1.5)
        hold on
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        plot(uav(n).pose.position.Time(T2.pose), ...
                uav(n).pose.position.Data(T2.pose,1), ...
                        'Color',green2,'Linewidth',1.5)
        hold on
        plot(trajectory(n).position.Time(T2.ref.pose), ...
                trajectory(n).position.Data(T2.ref.pose,1), ...
                            'Color',green1,'Linewidth',1.5)
        hold on
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        plot(uav(n).pose.position.Time(T3.pose), ...
                uav(n).pose.position.Data(T3.pose,1), ...
                            'Color',blue1,'Linewidth',1.5)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        title( strcat( ['Vehicle', ' ', num2str(uav(n).id)] ), ...
                                                    'Fontsize', 11 );
        ylabel('x [m]')
        hold on
        grid on

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        subplot(3,1,2);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        plot(uav(n).pose.position.Time(T1.pose), ...
                uav(n).pose.position.Data(T1.pose,2), ...
                        'Color',purple2,'Linewidth',1.5)
        hold on
        plot(trajectory(n).position.Time(T1.ref.pose), ...
                trajectory(n).position.Data(T1.ref.pose,2),...
                            'Color',purple1,'Linewidth',1.5)
        hold on
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        plot(uav(n).pose.position.Time(T2.pose), ...
                uav(n).pose.position.Data(T2.pose,2),...
                            'Color',green2,'Linewidth',1.5)
        hold on
        plot(trajectory(n).position.Time(T2.ref.pose), ...
                trajectory(n).position.Data(T2.ref.pose,2), ...
                            'Color',green1,'Linewidth',1.5)
        hold on
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        plot(uav(n).pose.position.Time(T3.pose),...
                uav(n).pose.position.Data(T3.pose,2),...
                        'Color',blue1,'Linewidth',1.5)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        ylabel('y [m]')
        hold on
        grid on

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        subplot(3,1,3);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        plot(uav(n).pose.position.Time(T1.pose),...
                uav(n).pose.position.Data(T1.pose,3),...
                    'Color',purple2,'Linewidth',1.5)
        hold on
        plot(trajectory(n).position.Time(T1.ref.pose), ...
                trajectory(n).position.Data(T1.ref.pose,3), ...
                            'Color',purple1,'Linewidth',1.5)
        hold on
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        p1 = plot(uav(n).pose.position.Time(T2.pose), ...
                    uav(n).pose.position.Data(T2.pose,3), ...
                            'Color',green2,'Linewidth',1.5);
        hold on
        p2 = plot(trajectory(n).position.Time(T2.ref.pose), ...
                    trajectory(n).position.Data(T2.ref.pose,3), ...
                                'Color',green1,'Linewidth',1.5);
        hold on
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        plot(uav(n).pose.position.Time(T3.pose), ...
                uav(n).pose.position.Data(T3.pose,3),...
                        'Color',blue1,'Linewidth',1.5)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        ylabel('z [m]')
        xlabel('Time [s]')
        legend([p1 p2],{'Observed','Trajectory'},'location','best');
        hold on
        grid on

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % LINEAR VELOCITY
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        figure
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        subplot(3,1,1)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        plot(uav(n).vel.lin.Time(T1.vel) ,...
                uav(n).vel.lin.Data(T1.vel,1), ...
                    'Color',purple2,'Linewidth',1.5)
        hold on
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        plot(uav(n).vel.lin.Time(T2.vel), ...
                uav(n).vel.lin.Data(T2.vel,1), ...
                    'Color',green2,'Linewidth',1.5)
        hold on
        plot(trajectory(n).velocity.Time, ...
                trajectory(n).velocity.Data(:,1), ...
                    'Color',green1,'Linewidth',1.5)
        hold on
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        plot(uav(n).vel.lin.Time(T3.vel), ...
                uav(n).vel.lin.Data(T3.vel,1), ...
                    'Color',blue1,'Linewidth',1.5)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        title( strcat( ['Vehicle', ' ', num2str(uav(n).id)] ), ... 
                                                    'Fontsize', 11 );
        ylabel('v_x [m/s]')
        hold on
        grid on

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        subplot(3,1,2)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        plot(uav(n).vel.lin.Time(T1.vel), ...
                uav(n).vel.lin.Data(T1.vel,2), ...
                    'Color',purple2,'Linewidth',1.5)
        hold on
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        plot(uav(n).vel.lin.Time(T2.vel), ...
                uav(n).vel.lin.Data(T2.vel,2), ...
                    'Color',green2,'Linewidth',1.5)
        hold on
        plot(trajectory(n).velocity.Time,...
                trajectory(n).velocity.Data(:,2), ...
                    'Color',green1,'Linewidth',1.5)
        hold on
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        plot(uav(n).vel.lin.Time(T3.vel), ...
                uav(n).vel.lin.Data(T3.vel,2), ...
                    'Color',blue1,'Linewidth',1.5)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        ylabel('v_y [m/s]')
        hold on
        grid on

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        subplot(3,1,3)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        plot(uav(n).vel.lin.Time(T1.vel), ...
                uav(n).vel.lin.Data(T1.vel,3), ...
                    'Color',purple2,'Linewidth',1.5)
        hold on
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        p1 = plot(uav(n).vel.lin.Time(T2.vel), ...
                    uav(n).vel.lin.Data(T2.vel,3), ...
                        'Color',green2,'Linewidth',1.5);
        hold on
        p2 = plot(trajectory(n).velocity.Time,...
                    trajectory(n).velocity.Data(:,3), ...
                        'Color',green1,'Linewidth',1.5);
        hold on
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        plot(uav(n).vel.lin.Time(T3.vel), ...
                uav(n).vel.lin.Data(T3.vel,3), ...
                    'Color',blue1,'Linewidth',1.5)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        ylabel('v_z [m/s]')
        xlabel('Time [s]')
        legend([p1 p2],{'Observed','Trajectory'},'location','best');
        grid on

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % THROTTLE
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        figure
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        p1 = plot(uav(n).cmd.thr.Time, ...
                    uav(n).cmd.thr.Data, ...
                        'Color',green3,'Linewidth',1.5);
        hold on
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        p2 = plot(trajectory(n).throttle.Time, ...
                    trajectory(n).throttle.Data, ...
                        'Color',green1,'Linewidth',1.5);
        hold on
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%      
        title( strcat( ['Vehicle', ' ', num2str(uav(n).id)] ), ...
                                                    'Fontsize', 11 );
        ylabel('Throttle')
        xlabel('Time [s]')
        legend([p1 p2],{'Input','Trajectory'},'location','best');
        grid on

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % ORIENTATION (EULER ANGLES)
        % EDIT ACCORDING TO THE CONTROL INPUTS CHOSEN
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        figure
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        subplot(3,1,1)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        plot(uav(n).pose.euler.Time(T1.pose),...
                uav(n).pose.euler.Data(T1.pose,1), ...
                    'Color',purple2,'Linewidth',1.5);
        hold on
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        plot(uav(n).cmd.euler.Time, ...
                uav(n).cmd.euler.Data(:,1), ...
                    'Color',green3,'Linewidth',1.5); % ROLL INPUT
        hold on
        plot(uav(n).pose.euler.Time(T2.pose), ...
            uav(n).pose.euler.Data(T2.pose,1), ...
                'Color',green2,'Linewidth',1.5);
        hold on
        plot(trajectory(n).euler.Time, ...
                trajectory(n).euler.Data(:,1), ...
                    'Color',green1,'Linewidth',1.5)
        hold on
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        plot(uav(n).pose.euler.Time(T3.pose), ...
                uav(n).pose.euler.Data(T3.pose,1), ...
                    'Color',blue1,'Linewidth',1.5);
        hold on
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        title( strcat( ['Vehicle', ' ', num2str(uav(n).id)] ), ...
                                                    'Fontsize', 11 );
        ylabel('\phi [deg]')
        hold on
        grid on

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        subplot(3,1,2)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        plot(uav(n).pose.euler.Time(T1.pose), ...
                uav(n).pose.euler.Data(T1.pose,2), ...
                        'Color',purple2,'Linewidth',1.5);
        hold on
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        plot(uav(n).cmd.euler.Time, ...
                uav(n).cmd.euler.Data(:,2), ...
                    'Color',green3,'Linewidth',1.5); % PITCH INPUT
        hold on
        plot(uav(n).pose.euler.Time(T2.pose), ...
                uav(n).pose.euler.Data(T2.pose,2), ...
                        'Color',green2,'Linewidth',1.5);
        hold on
        plot(trajectory(n).euler.Time, ...
                trajectory(n).euler.Data(:,2), ...
                    'Color',green1,'Linewidth',1.5)
        hold on
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        plot(uav(n).pose.euler.Time(T3.pose), ...
                uav(n).pose.euler.Data(T3.pose,2), ...
                        'Color',blue1,'Linewidth',1.5);
        hold on
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        ylabel('\theta [deg]')
        hold on
        grid on

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        subplot(3,1,3)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        plot(uav(n).pose.euler.Time(T1.pose), ...
                uav(n).pose.euler.Data(T1.pose,3), ...
                    'Color',purple2,'Linewidth',1.5);
        hold on
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        p1 = plot(uav(n).cmd.euler.Time, ...
                    uav(n).cmd.euler.Data(:,3), ...
                        'Color',green3,'Linewidth',1.5); % YAW INPUT
        hold on
        p2 = plot(uav(n).pose.euler.Time(T2.pose), ...
                    uav(n).pose.euler.Data(T2.pose,3), ...
                            'Color',green2,'Linewidth',1.5);
        hold on
        p3 = plot(trajectory(n).euler.Time, ...
                    trajectory(n).euler.Data(:,3), ...
                        'Color',green1,'Linewidth',1.5);
        hold on
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        plot(uav(n).pose.euler.Time(T3.pose), ...
                uav(n).pose.euler.Data(T3.pose,3), ...
                        'Color',blue1,'Linewidth',1.5);
        hold on
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        ylabel('\psi [deg]')
        xlabel('Time [s]')
        legend([p1 p2 p3],{'Input','Observed','Trajectory'},'location','best');
    %     legend([p2 p3],{'Observed','Trajectory'},'location','best');
        hold on
        grid on

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % ANGULAR VELOCITY
        % EDIT ACCORDING TO THE CONTROL INPUTS CHOSEN
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        figure
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        subplot(3,1,1)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        plot(uav(n).vel.ang.Time(T1.vel), ...
                uav(n).vel.ang.Data(T1.vel,1),...
                    'Color',purple2,'Linewidth',1.5)
        hold on
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %     plot(uav(n).cmd.rates.Time, ...
    %             uav(n).cmd.rates.Data(:,1), ...
    %                 'Color',green3,'Linewidth',1.5); % ROLL RATE INPUT
    %     hold on
        plot(uav(n).vel.ang.Time(T2.vel), ...
                uav(n).vel.ang.Data(T2.vel,1), ...
                    'Color',green2,'Linewidth',1.5)
        hold on
        plot(trajectory(n).rates.Time, ...
                trajectory(n).rates.Data(:,1), ...
                    'Color',green1,'Linewidth',1.5)
        hold on
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        plot(uav(n).vel.ang.Time(T3.vel), ...
                uav(n).vel.ang.Data(T3.vel,1), ...
                    'Color',blue1,'Linewidth',1.5)
        hold on
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        title( strcat( ['Vehicle', ' ', num2str(uav(n).id)] ), ...
                                                    'Fontsize', 11 );
        ylabel('p [rad/s]')
        grid on

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        subplot(3,1,2)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        plot(uav(n).vel.ang.Time(T1.vel), ...
                uav(n).vel.ang.Data(T1.vel,2), ...
                    'Color',purple2,'Linewidth',1.5)
        hold on
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %     plot(uav(n).cmd.rates.Time, ...
    %             uav(n).cmd.rates.Data(:,2), ...
    %                 'Color',green3,'Linewidth',1.5); % PITCH RATE INPUT
    %     hold on
        plot(uav(n).vel.ang.Time(T2.vel), ... 
                uav(n).vel.ang.Data(T2.vel,2), ...
                    'Color',green2,'Linewidth',1.5)
        hold on
        plot(trajectory(n).rates.Time, ...
                trajectory(n).rates.Data(:,2), ...
                    'Color',green1,'Linewidth',1.5)
        hold on
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        plot(uav(n).vel.ang.Time(T3.vel), ...
                uav(n).vel.ang.Data(T3.vel,2), ...
                    'Color',blue1,'Linewidth',1.5)
        hold on
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        ylabel('q [rad/s]')
        grid on

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        subplot(3,1,3)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        plot(uav(n).vel.ang.Time(T1.vel), ...
                uav(n).vel.ang.Data(T1.vel,3),...
                    'Color',purple2,'Linewidth',1.5)
        hold on
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %     p1 = plot(uav(n).cmd.rates.Time, ...
    %                 uav(n).cmd.rates.Data(:,3), ...
    %                     'Color',green3,'Linewidth',1.5); % YAW RATE INPUT
    %     hold on
        p2 = plot(uav(n).vel.ang.Time(T2.vel), ...
                    uav(n).vel.ang.Data(T2.vel,3), ...
                        'Color',green2,'Linewidth',1.5);
        hold on
        p3 = plot(trajectory(n).rates.Time, ...
                    trajectory(n).rates.Data(:,3), ...
                        'Color',green1,'Linewidth',1.5);
        hold on
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        plot(uav(n).vel.ang.Time(T3.vel), ...
                uav(n).vel.ang.Data(T3.vel,3), ...
                    'Color',blue1,'Linewidth',1.5)
        hold on
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        ylabel('r [rad/s]')
        xlabel('Time [s]')
    %     legend([p1 p2 p3],{'Input','Observed','Trajectory'},'location','best');
        legend([p2 p3],{'Observed','Trajectory'},'location','best');
        grid on
    end

    

end