function vehicles = setVehicles(vehicles)
% Sets the parameters of 'vehicle' structs according to the field 'type'.
% 
% This field indicates what is the name of the model employed to run
% experiments and from which flight data was collected and stored in the
% selected rosbag. This function as is assumes there exist only two
% different models, either 'iris' (default quadrotor, IRIS 3DR, in PX4 SITL 
% simulations in Gazebo) or 'intel' (short for Intel Aero RTF).
% 
% With each 'vehicle' its mass (in kg) and its 
% throttle-vs-thrust-vs-airspeed curve are associated, which are then used 
% to compute, for instance, angular velocities and throttle reference 
% trajectories. The throttle-vs-thrust curves 
% 
% The struct 'vehicle' also contains a field named 'cad'. This field is in
% itself also a struct with fields V (vertices), F (faces), and facecolors.
% These fiels are then used to draw vehicles in animations (check the
% built-in function PATCH). By default, this function models quadrotors in
% 3D as parallelepipeds.
% 
% Input:
% - vehicles: Array of 'vehicle' structs (use the round brackets operator 
%             to access the elements of the array). When using this
%             function each 'vehicle' must have the field 'type' specified.
% 
% Output:
% - vehicles: Array of 'vehicle' structs with all fields defined.
% 
% JOAO PINTO (2021-09-21)

    N = length(vehicles);
    clrs = jet(N);

    for k = 1:N
    
        if strcmp(vehicles(k).type,'iris')

            vehicles(k).mass = 1.52;
            
            % 'x' = thrust, 'y' = airspeed
            vehicles(k).thr_curve = ...
                @(x,y) ( x./(1 - sqrt( sum( y.^2, 2 ) ) / 25 ) ...
                            - 1.52*9.80665 ) / ( 2*34.068*0.561 + 7.1202 ) + 0.561;

        elseif strcmp(vehicles(k).type,'intel')

            vehicles(k).mass = 1.35;
            
            % 'x' = thrust
            vehicles(k).thr_curve = ...
                            @(x,y) ( tan( (x - 10.37)/8.84 ) + 1.478 )/2.995;
        end

        vehicles(k).cad = genBox( clrs(k,:) );
        vehicles(k).id = k;
    
    end
    
end

function box = genBox(colour)

    lxy = 0.25;
    lz = 0.15;

    % Vertices (V)
    V = [...
         -lxy -lxy -lz/2;... %1
         -lxy lxy -lz/2;... %2
         lxy lxy -lz/2;... %3
         lxy -lxy -lz/2;... %4
         lxy -lxy lz/2;... %5 
         lxy lxy lz/2;... %6
         -lxy lxy lz/2;... %7
         -lxy -lxy lz/2;... %8
        ];
    
    % Faces
    F = [...
            1 2 3 4; % box bottom plane
            4 5 6 3; % box +x plane
            6 7 2 3; % box +y plane
            7 8 1 2; % box -x plane
            8 1 4 5; % box -y plane
            5 6 7 8; % box top plane
        ];
    
    % Face Colours
    facecolors = repmat(colour,size(F,1),1);
    
    box.V = V;
    box.F = F;
    box.facecolors = facecolors;
end

