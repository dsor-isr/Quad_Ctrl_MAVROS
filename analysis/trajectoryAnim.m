function trajectoryAnim(ts,play_speed,t_start,t_finish,uav,trajectory)
% Displays an animation of the UAVs tracking the pre-defined trajectory 
% using the information gathered in the selected rosbag.
% 
% Inputs:
% - ts: Sampling time of the controller, or equivalently, the inverse of
%       the working frequency (in seconds);
% 
% - play_speed: Value that controlls the speed at which the animation is
%               played (in per second, s^{-1});
% 
% - t_start: Time instant from which the animation begins (this value is
%            defined in the trajectory timescale, i.e., t = 0 corresponds
%            to the beginning of the trajectory, while t = t*, with t* > 0,
%            to the end of the considered trajectory);
% 
% - t_finish: Time instant at which the animation ends (for this function
%             to work choose t_finish > t_start);
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
% Once this function is run, a new figure opens in full screen.
% JOAO PINTO (2021-09-21)

    f = figure('visible','off');
    ax = axes(f);

    L = length(uav);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
%     idx_v = zeros(1,L);
%     fin_v = zeros(1,L);
    
    w = zeros(1,L+1);
    w(1) = 0;
    
    x = struct('idx',[],'val',[]);
    sh = repmat(x,2,L);
        
    for n = 1:L
        y.idx = ...
            find( abs( uav(n).pose.position.Time - ...
                        uav(n).pose.sh(2).val - t_start ) <= 3*ts );
                    
        z.idx = ...
            find( abs( uav(n).pose.position.Time - ...
                        uav(n).pose.sh(2).val - t_finish ) <= 3*ts );
                    
        sh(1,n).idx = y.idx( round( length( y.idx )/2 ) );
        sh(2,n).idx = z.idx( round( length( z.idx )/2 ) );
        
        sh(1,n).val = round( uav(n).pose.position.Time( sh(1,n).idx ), 2 );
        sh(2,n).val = round( uav(n).pose.position.Time( sh(2,n).idx ), 2 );
        
        w(n+1) = w(n) + sh(2,n).idx - sh(1,n).idx + 1;
    end
    
    q = zeros(w(end),3);
    
    for n = 1:L

        K.pose = sh(1,n).idx:sh(2,n).idx;
        K.ref = round( sh(1,n).val/ts+1:sh(2,n).val/ts+1 );

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % 3D Trajectory Plot
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        plot3(ax, trajectory(n).position.Data(K.ref,1), ...
                    trajectory(n).position.Data(K.ref,2), ...
                        trajectory(n).position.Data(K.ref,3), ...
                                'LineStyle','--','Linewidth',1.5)
        hold on
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        xlabel('x [m]')
        ylabel('y [m]')
        zlabel('z [m]')
        axis equal
        hold on
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        

%         fin_v(n) = sh(2,n).idx;
%         idx_v(n) = sh(1,n).idx;
        
        q(w(n)+1:w(n+1),:) = uav(n).pose.position.Data(K.pose,:);
        
    end
    
    set(f,'Units','normalized','OuterPosition',[0 0 1 1])
    
    e = 0.5;
    
    axis equal
    grid on
    
    t = t_start:ts:t_finish;
    lt = length(t);
    
    k = 1;
    
%     [M_val, M_idx] = max(fin_v);
    quad_h = gobjects(1,L);
    traj_h = gobjects(1,L);
    
    set(f,'visible','on')
    pause(1)
    
%     while idx_v(M_idx) <= M_val
    while k <= lt
        
        for n = 1:L
                       
            x = find( abs( uav(n).pose.position.Time - ...
                        uav(n).pose.sh(2).val - t(k) ) <= 3*ts );
                    
            if isempty(x)
                continue
            end
                    
            x = x( round( length( x )/2 ) );
            
%             if ( idx_v(n) <= fin_v(n) )
                
                set(ax,'XLim',[min(q(:,1))-e,max(q(:,1))+e]);
                set(ax,'YLim',[min(q(:,2))-e,max(q(:,2))+e]);
                set(ax,'ZLim',[min(q(:,3))-e,max(q(:,3))+e]);
                
%                 pose = [ uav(n).pose.position.Data(idx_v(n),:) ...
%                                     uav(n).pose.rotm.Data(idx_v(n),:) ];

                pose = [ uav(n).pose.position.Data(x,:) ...
                                    uav(n).pose.rotm.Data(x,:) ];
                
                [quad_h(n),traj_h(n)] = ...
                    drawQuad(ax, uav(n).cad.V, uav(n).cad.F, ...
                        uav(n).cad.facecolors, pose, quad_h(n),traj_h(n));
                    
%                 idx_v(n) = idx_v(n) + 1;
                
%             end
            
        end
        
        k = k + 1;
        
        pause(1/play_speed);
        
    end
    
end

function [quad_h,traj_h] = drawQuad(ax,V,F,facecolors,pose,quad_h,traj_h)

    p = pose(1:3);
    R = reshape(pose(4:end)',3,3);
    
    V = rotate(V,R);
    V = translate(V,p);
      
    if ~isprop(quad_h,'Vertices') && ~isprop(traj_h,'Xdata')
        
        quad_h = patch(ax, 'Vertices', V, 'Faces', F,...
                                'FaceVertexCData',facecolors,...
                                                    'FaceColor','flat');
        hold on
        traj_h = plot3(ax, p(1), p(2), p(3),'Linewidth',1.5);
        
        grid on
        axis equal
        
    else
        set(quad_h,'Vertices',V,'Faces',F);
        set(traj_h,'Xdata',[get(traj_h,'Xdata'),p(1)]);
        set(traj_h,'Ydata',[get(traj_h,'Ydata'),p(2)]);
        set(traj_h,'Zdata',[get(traj_h,'Zdata'),p(3)]);

    end
  
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function V = rotate(V,R)
    
    V = V*R';
  
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function V = translate(V,p)

  V = V + repmat(p,size(V,1),1);
  
end
