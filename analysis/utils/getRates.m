function rates = getRates(mass,j,dyaw,R,T)
% Obtain angle rates from jerk, orientation, thrust, and the 1st time derivative of yaw.
% 
% Given a single linear jerk vector, a value for the 1st time derivative of
% the yaw angle, a single rotation matrix, and a value for thrust,
% 
%     h = mass*(jerk - (jerk'*zb)*zb)/T;
% 
%     p = -h'*yb,
%     q = h'*xb,
%     r = dyaw*(zw'*zb);
% 
%     rates = [p q r];
%
% where xb, yb, and zb denote the body frame basis vectors, and zw the
% world frame vector pointing in the z direction, zw = [0 0 1];
% 
% This function from N linear jerk vectors, N values of the first time
% derivative of the yaw angle, N rotation matrices, and N values of thrust
% returns N angular velocity vectors, expressed in the body frame.
% 
% Input:
% - mass: Vehicle mass in kg,
% 
% - j: (N x 3) matrix, where each column corresponds to the Cartesian
%      components of the N linear jerk vectors across the rows of this 
%      matrix;
% 
% - dyaw: column vector with N values specifying the first time derivative
%         of the yaw angle;
% 
% - R: (3 x 3 x N) tensor with N rotation matrix, R(:,:,j) corresponds to
%      the rotation matrix j, with 1 <= j <= N;
% 
% - T: column vector with N thrust values.
% 
% Output:
% - rates: (N x 3) matrix, where each row is an angular velocity vector.
%          The first column corresponds to the roll rate, the second, to
%          the pitch rate, and the third, to the yaw rate.
% 
% JOAO PINTO (2021-09-21)


    xb = squeeze( R(:,1,:) );
    xb = xb';
    
    yb = squeeze( R(:,2,:) );
    yb = yb';

    zb = squeeze( R(:,3,:) );
    zb = zb';
    
    h = mass*( j - sum( j.*zb, 2 ).*zb )./T;
    p = -sum( h.*yb, 2 );
    q = sum( h.*xb, 2 );
    
    r = dyaw.*zb(:,3);
    
    rates = [p q r];

end

