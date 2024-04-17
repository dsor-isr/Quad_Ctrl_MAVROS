function R = acc2R(acc,yaw)
% Obtain rotation matrices from a collection of acceleration and yaw vectors.
% 
% Given a single linear acceleration vector and a yaw angle value, the
% correspoding rotation matrix can be obtaine the following equations,
% 
%     acc(3) = acc(3) + 9.8;
%     zb = acc/norm(acc);
%     
%     xc = [cos(yaw) sin(yaw) 0]';
%     yb = skew(zb)*xc;
%     yb = yb/norm(yb);
%     
%     xb = skew(yb)*zb;
%     
%     R = [xb yb zb];
% 
% This function from N linear acceleration vector and N yaw angle values,
% returns N rotation matrices.
% 
% Input:
% - acc: (N x 3) matrix, where each column corresponds to the Cartesian
%        components of the linear acceleration vector;
% - yaw: column vector with length N specifying the yaw angle values.
% 
% Output:
% - R: (3 x 3 x N) tensor, where R(:,:,j) corresponds to the rotation
%      matrix obatined from acc(j,:) and yaw(j).
% 
% JOAO PINTO (2021-09-21)

    acc(:,3) = acc(:,3) + 9.8;
    zb = acc./sqrt( sum( acc.^2, 2 ) );
    
    xc = [cos(yaw) sin(yaw) zeros(length(yaw),1)];
    xc = reshape(xc',1,3,[]);
    
    yb = squeeze( sum( skew(zb).*xc, 2 ) );
    yb = yb';
    yb = yb./sqrt( sum( yb.^2, 2 ) );
    
    zb_aux = reshape(zb',1,3,[]);
    xb = squeeze( sum( skew(yb).*zb_aux, 2 ) );
    xb = xb';
    
    R_aux = [xb yb zb];
    R = reshape(R_aux',3,3,[]);
    
end

