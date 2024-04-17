function trajectoryPlot3D(ts,vt,t_start,t_finish,uav,trajectory)
% Draws observed and reference position trajectories in 3D.
% 
% Multiple drawings of the vehicles are placed over the observed
% trajectories, taking into account their actual orientaion at specific 
% time instances.
% 
% Inputs:
% - ts: Sampling time of the controller, or equivalently, the inverse of
%       the working frequency (in seconds);
% 
% - vt: Array of doubles defining the time instant at which the vehicles
%       are drawn. Every element of this array must satisfy t_start <=
%       vt(j) <= t_end, where j = 1,2,...,length(vt). At time instant vt(j)
%       UAV #n is depicted at uav(n).pose.position.Data(k) with
%       orientation, relative to the world frame, uav.pose.rotm.Data(k). 
% 
% - t_start: Time instant from which the trajectories are drawn (this value 
%            is defined in the trajectory timescale, i.e., t = 0 corresponds
%            to the beginning of the trajectory, while t = t*, with t* > 0,
%            to the end of the considered trajectory);
% 
% - t_finish: Time instant until which the trajectories are drawn (for this 
%             function to work choose t_finish > t_start);
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

    f = figure('visible','off');
    ax = axes(f);
    
    L = length(uav);
    
    x = struct('idx',[],'val',[]);
    sh = repmat(x,2,1);
    
    for n = 1:L
        
        sh(1).idx = ...
            find( abs( uav(n).pose.position.Time - ...
                        uav(n).pose.sh(2).val - t_start ) <= 3*ts );
                    
        sh(2).idx = ...
            find( abs( uav(n).pose.position.Time - ...
                        uav(n).pose.sh(2).val - t_finish ) <= 3*ts );
                    
        sh(1).idx = sh(1).idx( round( length( sh(1).idx )/2 ) );
        sh(2).idx = sh(2).idx( round( length( sh(2).idx )/2 ) );
        
        sh(1).val = round( uav(n).pose.position.Time( sh(1).idx ), 2 );
        sh(2).val = round( uav(n).pose.position.Time( sh(2).idx ), 2 );
    
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        K.pose = sh(1).idx:sh(2).idx;
        K.ref = round( sh(1).val/ts+1:sh(2).val/ts+1 );

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % 3D Trajectory Plot
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        plot3(ax, uav(n).pose.position.Data(K.pose,1), ...
                    uav(n).pose.position.Data(K.pose,2), ...
                        uav(n).pose.position.Data(K.pose,3), ...
                                'LineStyle','-','Linewidth',1.5)
        hold on
        plot3(ax, trajectory(n).position.Data(K.ref,1), ...
                    trajectory(n).position.Data(K.ref,2), ...
                        trajectory(n).position.Data(K.ref,3), ...
                            'LineStyle','--','Linewidth',1.5)
        hold on
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        J = length(vt);

        for j = 1:J
            
            r = find( abs( uav(n).pose.position.Time - ...
                                uav(n).pose.sh(2).val - vt(j) ) <= 3*ts );

            k = r( round(length(r)/2) );

            p = uav(n).pose.position.Data(k,:);
            R = reshape( uav(n).pose.rotm.Data(k,:)', 3, 3 );

            drawQuad(ax, uav(n).cad.V, uav(n).cad.F, ...
                                uav(n).cad.facecolors, p, R);
                        
        end
    
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    xlabel('x [m]')
    ylabel('y [m]')
    zlabel('z [m]')
    axis equal
    hold on
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    grid on
    
    set(f,'visible','on');
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function drawQuad(ax,V,F,facecolors,p,R)
        
    V = rotate(V,R);
    V = translate(V,p);
        
    patch(ax, 'Vertices', V, 'Faces', F,...
                'FaceVertexCData',facecolors,...
                                'FaceColor','flat');
             
    hold on
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function V = rotate(V,R)
    
    V = V*R';
  
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function V = translate(V,p)

  V = V + repmat(p,size(V,1),1);
  
end

